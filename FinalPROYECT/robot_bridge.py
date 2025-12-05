"""
Robot Bridge Server - Conecta Browser con OpenAI Realtime API y MCP Robot Server

Este servidor actúa como puente bidireccional entre:
1. Browser del usuario (vía WebSocket)
2. OpenAI Realtime API (vía WebSocket)
3. MCP Robot Server (vía SSE - mcp_server.py)

Arquitectura del flujo de datos:
    Browser (HTML/JS) <--WebSocket--> robot_bridge.py <--WebSocket--> OpenAI Realtime
                                             |
                                           SSE/HTTP
                                             |
                                        mcp_server.py <--> robot_service.py <--> Robot Create3

Autores:
    - Alan Ariel Salazar
    - Yago Ramos Sánchez

Institución:
    Universidad Intercontinental de la Empresa (UIE)

Profesor:
    Eladio Dapena

Asignatura:
    Robots Autónomos
"""

import os
import asyncio
import json
import sys
import base64
import io
from pathlib import Path
from typing import Optional

import websockets
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
from mcp import ClientSession
from mcp.client.sse import sse_client
import uvicorn
from dotenv import load_dotenv

# Imports para conversión de audio WebM → PCM16
try:
    from pydub import AudioSegment
    AUDIO_CONVERSION_AVAILABLE = True
except ImportError:
    AUDIO_CONVERSION_AVAILABLE = False
    print("[WARNING] pydub no está instalado. La conversión de audio WebM→PCM16 no estará disponible.", file=sys.stderr)
    print("[WARNING] Instala con: pip install pydub", file=sys.stderr)

# Cargar variables de entorno
load_dotenv()

# Configuración
OPENAI_API_KEY = os.getenv('OPENAI_API_KEY')
OPENAI_MODEL = 'gpt-4o-realtime-preview-2024-12-17'
OPENAI_REALTIME_URL = f'wss://api.openai.com/v1/realtime?model={OPENAI_MODEL}'
PORT = 8000
HOST = '0.0.0.0'

# Configuración MCP Server (SSE)
MCP_SERVER_URL = os.getenv('MCP_SERVER_URL', 'http://localhost:8001/sse')


def get_system_instructions(node_list: Optional[str] = None) -> str:
    """
    Genera las instrucciones del sistema para OpenAI, incluyendo opcionalmente
    la lista de nodos disponibles para reducir alucinaciones.
    """
    base_instructions = """Eres un asistente  que controla un iRobot Create 3 mediante navegación topológica.

Tienes acceso a herramientas MCP para controlar el robot. Las herramientas disponibles son:

1. list_available_locations() - Lista todos los nodos/ubicaciones del mapa topológico
2. navigate_robot(destination_id, origin_id) - Navega el robot a un nodo destino
3. emergency_stop() - Detiene el robot inmediatamente
4. get_robot_status() - Consulta el estado actual del robot (MUY IMPORTANTE: incluye current_node)"""

    if node_list:
        base_instructions += f"""

MAPA DEL ROBOT (Nodos disponibles):
{node_list}

IMPORTANTE: Estos son los ÚNICOS nodos válidos. NO inventes IDs o nombres que no estén en esta lista."""

    base_instructions += """

REGLAS CRÍTICAS PARA NAVEGACIÓN:
1. SIEMPRE necesitas DOS valores para navegar: origin_id (de dónde sale) y destination_id (a dónde va)
2. Si el usuario NO especifica el nodo origen EXPLÍCITAMENTE con un número:
   - PRIMERO llama a get_robot_status() para obtener el current_node
   - LUEGO usa ese current_node como origin_id
3. Si el usuario dice "el último", "donde está", "donde quedó", "su posición actual":
   - DEBES llamar a get_robot_status() primero
   - Usa el current_node de la respuesta como origin_id
4. NUNCA llames a navigate_robot() sin tener AMBOS valores confirmados
5. Si el usuario dice "del nodo X al Y" → origin_id=X, destination_id=Y
6. Si el usuario solo dice "ve al nodo Y" sin especificar origen → llama get_robot_status() primero

Otras instrucciones:
- SIEMPRE usa los IDs numéricos exactos de los nodos
- Sé conciso en tus respuestas por voz - esto es una interfaz de audio

Ejemplos de uso CORRECTO:
- Usuario: "Llévame del nodo 0 al 5" → navigate_robot(destination_id=5, origin_id=0)
- Usuario: "Ve al nodo 3" → PRIMERO get_robot_status(), luego navigate_robot(destination_id=3, origin_id=<current_node>)
- Usuario: "Ahora ve al nodo 2" (después de otra navegación) → get_robot_status() para obtener current_node, luego navigate_robot

Contesta con un tono natural y cordial.
Recuerda: SIEMPRE verifica el nodo actual antes de navegar si el usuario no lo especifica."""
    
    return base_instructions


# FastAPI app
app = FastAPI(title="Robot Create3 Bridge Server")

# Servir archivos estáticos (HTML/CSS/JS)
app.mount("/static", StaticFiles(directory="static"), name="static")


def convert_webm_to_pcm16(webm_data: bytes) -> bytes:
    """Convierte audio WebM/Opus a PCM16 mono 24kHz para OpenAI Realtime API."""
    if not AUDIO_CONVERSION_AVAILABLE:
        raise Exception("pydub no está instalado. No se puede convertir audio.")
    
    try:
        audio = AudioSegment.from_file(io.BytesIO(webm_data), format="webm")
        if audio.channels > 1:
            audio = audio.set_channels(1)
        if audio.frame_rate != 24000:
            audio = audio.set_frame_rate(24000)
        audio = audio.set_sample_width(2)
        return audio.raw_data
    except Exception as e:
        print(f"[Audio] Error convirtiendo WebM a PCM16: {e}", file=sys.stderr)
        raise


@app.get("/")
async def root():
    """Redirigir a la interfaz web"""
    from fastapi.responses import FileResponse
    return FileResponse("static/index.html")


@app.get("/health")
async def health_check():
    """Health check endpoint"""
    return JSONResponse({
        "status": "ok",
        "service": "Robot Create3 Bridge",
        "openai_configured": OPENAI_API_KEY is not None,
        "mcp_server_url": MCP_SERVER_URL
    })


class BridgeSession:
    """
    Gestiona una sesión completa de bridge:
    - Conexión con el browser
    - Conexión con OpenAI Realtime
    - Conexión con MCP Robot Server (vía SSE)
    """
    
    def __init__(self, client_ws: WebSocket):
        self.client_ws = client_ws
        self.openai_ws: Optional[websockets.WebSocketClientProtocol] = None
        self.mcp_session: Optional[ClientSession] = None
        self.mcp_streams = None
        self.running = False
        self.telemetry_queue: asyncio.Queue = asyncio.Queue()
        self.node_list: Optional[str] = None
        
        # Control de respuestas de OpenAI
        self.response_in_progress = False
        self.pending_tool_calls: asyncio.Queue = asyncio.Queue()
        self._response_lock = asyncio.Lock()
        
    async def send_to_client(self, message_type: str, data: dict):
        """Enviar mensaje JSON al cliente browser"""
        try:
            await self.client_ws.send_json({
                "type": message_type,
                **data
            })
        except Exception as e:
            print(f"[Bridge] Error enviando a cliente: {e}", file=sys.stderr)
    
    async def log(self, message: str, level: str = "info"):
        """Enviar log al cliente"""
        await self.send_to_client("log", {
            "message": message,
            "level": level
        })
    
    async def run(self):
        """Ejecutar el bucle principal del bridge"""
        try:
            self.running = True
            await self.log("Iniciando bridge session...", "info")
            
            await self.connect_to_mcp()
            await self.connect_to_openai()
            await self.communication_loop()
            
        except asyncio.CancelledError:
            print("[Bridge] Sesión cancelada (cliente desconectado)", file=sys.stderr)
        except WebSocketDisconnect:
            print("[Bridge] Cliente desconectado", file=sys.stderr)
        except Exception as e:
            error_msg = f"Error en bridge session: {str(e)}"
            print(f"[Bridge] {error_msg}", file=sys.stderr)
            try:
                await self.send_to_client("error", {"message": error_msg})
            except:
                pass
            import traceback
            traceback.print_exc(file=sys.stderr)
        finally:
            self.running = False
            await self.cleanup()
    
    async def connect_to_mcp(self):
        """Conectar al servidor MCP del robot vía SSE"""
        await self.log("Conectando al servidor MCP del robot...", "info")
        await self.log(f"URL: {MCP_SERVER_URL}", "info")
        
        try:
            self.mcp_streams = sse_client(MCP_SERVER_URL)
            read_stream, write_stream = await self.mcp_streams.__aenter__()
            
            self.mcp_session = ClientSession(read_stream, write_stream)
            await self.mcp_session.__aenter__()
            await self.mcp_session.initialize()
            
            tools_response = await self.mcp_session.list_tools()
            self.mcp_tools = tools_response.tools
            
            await self.log(f"MCP conectado - {len(self.mcp_tools)} herramientas disponibles", "success")
            
            self.openai_tools = []
            for tool in self.mcp_tools:
                openai_tool = {
                    "type": "function",
                    "name": tool.name,
                    "description": tool.description or f"Herramienta MCP: {tool.name}",
                    "parameters": tool.inputSchema
                }
                self.openai_tools.append(openai_tool)
                await self.log(f"  - {tool.name}", "info")
            
            try:
                result = await self.mcp_session.call_tool("list_available_locations", {})
                if hasattr(result, 'content') and result.content:
                    for content_item in result.content:
                        if hasattr(content_item, 'text'):
                            self.node_list = content_item.text
                            await self.log("Lista de nodos obtenida para prompt de IA", "success")
                            break
            except Exception as e:
                await self.log(f"Advertencia: No se pudo obtener lista de nodos: {e}", "info")
                self.node_list = None
            
            await self.log("Sistema de telemetría mediante polling activado", "success")
            
        except Exception as e:
            error_msg = f"Error conectando a MCP: {str(e)}"
            await self.log(error_msg, "error")
            raise Exception(f"No se pudo conectar al servidor MCP en {MCP_SERVER_URL}. ¿Está ejecutándose mcp_server.py?")
    
    async def connect_to_openai(self):
        """Conectar a OpenAI Realtime API"""
        await self.log("Conectando a OpenAI Realtime API...", "info")
        
        if not OPENAI_API_KEY:
            raise ValueError("OPENAI_API_KEY no está configurada en el .env")
        
        headers = {
            "Authorization": f"Bearer {OPENAI_API_KEY}",
            "OpenAI-Beta": "realtime=v1"
        }
        
        self.openai_ws = await websockets.connect(
            OPENAI_REALTIME_URL,
            extra_headers=headers
        )
        
        await self.log("Conectado a OpenAI Realtime", "success")
        
        system_instructions = get_system_instructions(self.node_list)
        
        session_config = {
            "type": "session.update",
            "session": {
                "modalities": ["text", "audio"],
                "instructions": system_instructions,
                "voice": "alloy",
                "input_audio_format": "pcm16",
                "output_audio_format": "pcm16",
                "input_audio_transcription": {
                    "model": "whisper-1"
                },
                "turn_detection": {
                    "type": "server_vad",
                    "threshold": 0.5,
                    "prefix_padding_ms": 300,
                    "silence_duration_ms": 500
                },
                "tools": self.openai_tools,
                "tool_choice": "auto",
                "temperature": 0.8
            }
        }
        
        await self.openai_ws.send(json.dumps(session_config))
        await self.log("Sesión OpenAI configurada con herramientas MCP", "success")
        await self.send_to_client("status", {
            "message": "Conectado - ¡Puedes hablar ahora!",
            "status": "connected"
        })
    
    async def safe_request_response(self):
        """Solicita respuesta a OpenAI solo si no hay una en progreso"""
        async with self._response_lock:
            if not self.response_in_progress and self.openai_ws and not self.openai_ws.closed:
                try:
                    await self.openai_ws.send(json.dumps({"type": "response.create"}))
                    self.response_in_progress = True
                    print(f"[OpenAI] Respuesta solicitada", file=sys.stderr)
                except Exception as e:
                    print(f"[OpenAI] Error solicitando respuesta: {e}", file=sys.stderr)
    
    async def communication_loop(self):
        """Bucle principal de comunicación bidireccional"""
        
        async def browser_to_openai():
            """Browser → OpenAI (audio del usuario)"""
            try:
                while self.running:
                    try:
                        pcm16_data = await self.client_ws.receive_bytes()
                        
                        if len(pcm16_data) == 0:
                            continue
                        
                        if len(pcm16_data) % 2 != 0:
                            pcm16_data = pcm16_data[:len(pcm16_data) - 1]
                        
                        if self.openai_ws and not self.openai_ws.closed:
                            audio_base64 = base64.b64encode(pcm16_data).decode('utf-8')
                            await self.openai_ws.send(json.dumps({
                                "type": "input_audio_buffer.append",
                                "audio": audio_base64
                            }))
                        else:
                            print(f"[Audio] OpenAI WS cerrado, no se puede enviar audio", file=sys.stderr)
                    except WebSocketDisconnect:
                        print(f"[Audio] Cliente WebSocket desconectado", file=sys.stderr)
                        break
                    except websockets.exceptions.ConnectionClosed:
                        print(f"[Audio] Conexión OpenAI cerrada", file=sys.stderr)
                        break
                    except Exception as e:
                        if self.running:
                            print(f"[Audio] Error procesando audio: {e}", file=sys.stderr)
                    
            except Exception as e:
                if self.running:
                    print(f"[Audio] Error en browser→openai: {e}", file=sys.stderr)
            finally:
                print(f"[Audio] Bucle browser_to_openai terminando", file=sys.stderr)
        
        async def openai_to_browser_and_mcp():
            """OpenAI → Browser + MCP (respuestas y llamadas a herramientas)"""
            try:
                print(f"[OpenAI] Iniciando bucle de lectura de mensajes...", file=sys.stderr)
                async for message in self.openai_ws:
                    if not self.running:
                        break
                    
                    try:
                        event = json.loads(message)
                        event_type = event.get("type", "")
                        
                        # === TRANSCRIPCIONES DEL USUARIO ===
                        if event_type == "conversation.item.input_audio_transcription.completed":
                            transcript = event.get("transcript", "")
                            try:
                                await self.send_to_client("transcript", {
                                    "sender": "user",
                                    "content": transcript
                                })
                                await self.log(f"Usuario: {transcript}", "info")
                            except:
                                pass
                        
                        # === RESPUESTA INICIADA ===
                        elif event_type == "response.created":
                            async with self._response_lock:
                                self.response_in_progress = True
                            print(f"[OpenAI] Respuesta iniciada", file=sys.stderr)
                        
                        # === RESPUESTA COMPLETADA ===
                        elif event_type == "response.done":
                            async with self._response_lock:
                                self.response_in_progress = False
                            # Detalles de la respuesta para debug
                            response_obj = event.get("response", {})
                            output_items = response_obj.get("output", [])
                            status = response_obj.get("status", "unknown")
                            print(f"[OpenAI] Respuesta completada - status={status}, outputs={len(output_items)}", file=sys.stderr)
                            for item in output_items:
                                item_type = item.get("type", "?")
                                print(f"[OpenAI]   - Output: type={item_type}", file=sys.stderr)
                        
                        # === AUDIO DE RESPUESTA ===
                        elif event_type == "response.audio.delta":
                            audio_delta = event.get("delta", "")
                            if audio_delta:
                                try:
                                    audio_bytes = base64.b64decode(audio_delta)
                                    print(f"[OpenAI] Audio delta recibido: {len(audio_bytes)} bytes", file=sys.stderr)
                                    await self.client_ws.send_bytes(audio_bytes)
                                except Exception as e:
                                    print(f"[OpenAI] Error enviando audio: {e}", file=sys.stderr)
                        
                        # === TRANSCRIPCIÓN DEL ASISTENTE ===
                        elif event_type == "response.audio_transcript.done":
                            transcript = event.get("transcript", "")
                            if transcript:
                                try:
                                    await self.send_to_client("transcript", {
                                        "sender": "assistant",
                                        "content": transcript
                                    })
                                    await self.log(f"Robot: {transcript}", "success")
                                except:
                                    pass
                        
                        # === LLAMADAS A HERRAMIENTAS MCP ===
                        elif event_type == "response.function_call_arguments.done":
                            tool_name = event.get('name', 'unknown')
                            print(f"[OpenAI] Llamada a herramienta: {tool_name}", file=sys.stderr)
                            asyncio.create_task(self.handle_tool_call_safe(event))
                        
                        # === ERRORES ===
                        elif event_type == "error":
                            error_obj = event.get("error", {})
                            error_msg = error_obj.get("message", "Error desconocido")
                            error_type = error_obj.get("type", "unknown")
                            error_code = error_obj.get("code", "none")
                            print(f"[OpenAI] ERROR: type={error_type}, code={error_code}, msg={error_msg}", file=sys.stderr)
                            # Ignorar errores de "response in progress" silenciosamente
                            if "active response in progress" not in error_msg:
                                try:
                                    await self.log(f"Error OpenAI: {error_msg}", "error")
                                    await self.send_to_client("error", {"message": error_msg})
                                except:
                                    pass
                            else:
                                print(f"[OpenAI] Ignorando error de respuesta activa", file=sys.stderr)
                        
                        # === EVENTOS DE SESIÓN ===
                        elif event_type in ["session.created", "session.updated"]:
                            try:
                                await self.log(f"Sesión OpenAI: {event_type}", "info")
                            except:
                                pass
                        
                        # === EVENTOS DE VOZ (VAD) ===
                        elif event_type == "input_audio_buffer.speech_started":
                            print(f"[OpenAI] 🎤 Voz detectada - comenzando grabación", file=sys.stderr)
                        elif event_type == "input_audio_buffer.speech_stopped":
                            print(f"[OpenAI] 🎤 Voz terminada - procesando...", file=sys.stderr)
                        elif event_type == "input_audio_buffer.committed":
                            print(f"[OpenAI] 🎤 Audio enviado para transcripción", file=sys.stderr)
                                
                    except json.JSONDecodeError:
                        pass
                    except Exception as e:
                        if self.running:
                            print(f"[OpenAI] Error procesando evento: {e}", file=sys.stderr)
                
            except websockets.exceptions.ConnectionClosed as e:
                print(f"[OpenAI] Conexión cerrada: {e}", file=sys.stderr)
                self.running = False
            except Exception as e:
                if self.running:
                    print(f"[OpenAI] Error: {e}", file=sys.stderr)
                self.running = False
            finally:
                print(f"[OpenAI] Bucle terminando", file=sys.stderr)
        
        async def telemetry_polling():
            """Telemetría del Robot → Browser"""
            try:
                await asyncio.sleep(2)
                
                last_state = None
                last_mission_status = None
                
                while self.running:
                    try:
                        if not self.mcp_session:
                            await asyncio.sleep(1)
                            continue
                        
                        result = await self.mcp_session.call_tool("get_robot_status", {})
                        
                        if hasattr(result, 'content') and result.content:
                            for content_item in result.content:
                                if hasattr(content_item, 'text'):
                                    try:
                                        status_data = json.loads(content_item.text)
                                        
                                        try:
                                            await self.send_to_client("telemetry", {
                                                "position": status_data.get("position", {"x": 0, "y": 0, "theta": 0}),
                                                "state": status_data.get("state"),
                                                "current_node": status_data.get("current_node"),
                                                "target_node": status_data.get("target_node"),
                                                "last_message": status_data.get("last_message")
                                            })
                                            await self.send_to_client("robot_state", {"state": status_data})
                                        except:
                                            pass
                                        
                                        current_state = status_data.get("state")
                                        mission_completed = status_data.get("mission_completed", False)
                                        
                                        # Detectar finalización de misión
                                        if last_mission_status == False and mission_completed == True:
                                            try:
                                                await self.log(f"🎉 Misión completada: {status_data.get('last_message')}", "success")
                                                await self.send_to_client("mission_event", {
                                                    "event": "mission_completed",
                                                    "message": status_data.get("last_message"),
                                                    "destination": status_data.get("current_node")
                                                })
                                            except:
                                                pass
                                        
                                        # Detectar inicio de navegación
                                        if last_state != "NAVIGATING" and current_state == "NAVIGATING":
                                            try:
                                                await self.log(f"🚀 Navegación iniciada hacia nodo {status_data.get('target_node')}", "info")
                                            except:
                                                pass
                                        
                                        # Detectar errores
                                        if last_state != "ERROR" and current_state == "ERROR":
                                            try:
                                                await self.log(f"❌ Error: {status_data.get('last_message')}", "error")
                                            except:
                                                pass
                                        
                                        last_state = current_state
                                        last_mission_status = mission_completed
                                        
                                    except json.JSONDecodeError:
                                        pass
                        
                    except Exception as e:
                        if not self.running:
                            break
                    
                    # Polling cada 1 segundo (reducido de 0.5s para evitar conflictos durante navegación)
                    await asyncio.sleep(1.0)
                    
            except asyncio.CancelledError:
                pass
            except Exception as e:
                if self.running:
                    print(f"[Telemetry] Error: {e}", file=sys.stderr)
        
        print(f"[Bridge] Iniciando bucles de comunicación...", file=sys.stderr)
        results = await asyncio.gather(
            browser_to_openai(),
            openai_to_browser_and_mcp(),
            telemetry_polling(),
            return_exceptions=True
        )
        
        print(f"[Bridge] Bucles terminados", file=sys.stderr)
        for i, result in enumerate(results):
            if isinstance(result, Exception) and not isinstance(result, (asyncio.CancelledError, WebSocketDisconnect)):
                task_names = ["browser_to_openai", "openai_to_browser_and_mcp", "telemetry_polling"]
                print(f"[Bridge] Tarea {task_names[i]} error: {result}", file=sys.stderr)
    
    async def handle_tool_call_safe(self, event: dict):
        """Wrapper seguro para handle_tool_call"""
        try:
            await self.handle_tool_call(event)
        except Exception as e:
            print(f"[Tool] Error crítico: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc(file=sys.stderr)
    
    async def handle_tool_call(self, event: dict):
        """Manejar llamadas a herramientas MCP desde OpenAI"""
        tool_name = event.get("name", "")
        args_string = event.get("arguments", "{}")
        call_id = event.get("call_id", "")
        
        print(f"[Tool] Ejecutando {tool_name} (call_id: {call_id})", file=sys.stderr)
        
        try:
            args = json.loads(args_string)
            
            try:
                await self.log(f"Ejecutando herramienta: {tool_name}", "tool")
                await self.send_to_client("transcript", {
                    "sender": "tool",
                    "content": f"Ejecutando: {tool_name}({json.dumps(args, indent=2)})"
                })
            except:
                pass
            
            result_text = None
            try:
                if not self.mcp_session:
                    raise Exception("Sesión MCP no disponible")
                
                result = await asyncio.wait_for(
                    self.mcp_session.call_tool(tool_name, args),
                    timeout=60.0
                )
                
                try:
                    result_text = self.format_mcp_result(result)
                except:
                    result_text = str(result) if result else "Resultado vacío"
                
                try:
                    await self.log(f"Herramienta completada: {tool_name}", "success")
                except:
                    pass
                
                if tool_name == "get_robot_status" and hasattr(result, 'content'):
                    for content_item in result.content:
                        if hasattr(content_item, 'text'):
                            try:
                                status_data = json.loads(content_item.text)
                                await self.send_to_client("robot_state", {"state": status_data})
                            except:
                                pass
                                
            except asyncio.TimeoutError:
                result_text = json.dumps({"error": f"Timeout ejecutando {tool_name}"})
            except Exception as e:
                result_text = json.dumps({"error": f"Error: {str(e)}"})
            
            if result_text is None:
                result_text = json.dumps({"error": "Resultado no disponible"})
            
            # Enviar resultado a OpenAI
            if self.openai_ws and not self.openai_ws.closed:
                try:
                    if not isinstance(result_text, str):
                        result_text = json.dumps({"error": "Formato inválido"})
                    
                    await self.openai_ws.send(json.dumps({
                        "type": "conversation.item.create",
                        "item": {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": result_text
                        }
                    }))
                    
                    # Solo solicitar respuesta si no hay una activa
                    await self.safe_request_response()
                    
                except websockets.exceptions.ConnectionClosed:
                    print(f"[Tool] Conexión cerrada", file=sys.stderr)
                except Exception as e:
                    print(f"[Tool] Error enviando: {e}", file=sys.stderr)
            
        except json.JSONDecodeError as e:
            if self.openai_ws and not self.openai_ws.closed:
                try:
                    await self.openai_ws.send(json.dumps({
                        "type": "conversation.item.create",
                        "item": {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": json.dumps({"error": f"Error parseando argumentos: {e}"})
                        }
                    }))
                except:
                    pass
            
        except Exception as e:
            print(f"[Tool] Error: {e}", file=sys.stderr)
            if self.openai_ws and not self.openai_ws.closed:
                try:
                    await self.openai_ws.send(json.dumps({
                        "type": "conversation.item.create",
                        "item": {
                            "type": "function_call_output",
                            "call_id": call_id,
                            "output": json.dumps({"error": str(e)})
                        }
                    }))
                except:
                    pass
        
        finally:
            print(f"[Tool] {tool_name} finalizado", file=sys.stderr)
    
    def format_mcp_result(self, result) -> str:
        """Formatear resultado MCP para OpenAI"""
        if hasattr(result, 'content') and result.content:
            text_parts = []
            for item in result.content:
                if hasattr(item, 'text'):
                    text_parts.append(item.text)
            return "\n".join(text_parts) if text_parts else str(result)
        return str(result)
    
    async def cleanup(self):
        """Limpiar recursos de forma segura"""
        print("[Bridge] Limpiando recursos...", file=sys.stderr)
        
        try:
            await self.log("Cerrando conexiones...", "info")
        except:
            pass
        
        # Cerrar conexión OpenAI
        if self.openai_ws:
            try:
                if not self.openai_ws.closed:
                    await asyncio.wait_for(self.openai_ws.close(), timeout=2.0)
            except:
                pass
        
        # Cerrar sesión MCP
        if self.mcp_session:
            try:
                await asyncio.wait_for(
                    self.mcp_session.__aexit__(None, None, None),
                    timeout=2.0
                )
            except:
                pass
        
        # Cerrar streams MCP
        if self.mcp_streams:
            try:
                await asyncio.wait_for(
                    self.mcp_streams.__aexit__(None, None, None),
                    timeout=2.0
                )
            except:
                pass
        
        print("[Bridge] Limpieza completada", file=sys.stderr)


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """Endpoint WebSocket principal"""
    await websocket.accept()
    print(f"[Bridge] Nueva conexión WebSocket", file=sys.stderr)
    
    session = BridgeSession(websocket)
    
    try:
        await session.run()
    except Exception as e:
        print(f"[Bridge] Error en sesión: {e}", file=sys.stderr)
    finally:
        print(f"[Bridge] Sesión finalizada", file=sys.stderr)


if __name__ == "__main__":
    print("=" * 60, file=sys.stderr)
    print("ROBOT CREATE3 BRIDGE SERVER", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print(f"Servidor: http://{HOST}:{PORT}", file=sys.stderr)
    print(f"Interfaz Web: http://{HOST}:{PORT}/", file=sys.stderr)
    print(f"WebSocket: ws://{HOST}:{PORT}/ws", file=sys.stderr)
    print(f"Health Check: http://{HOST}:{PORT}/health", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    print(f"MCP Server URL: {MCP_SERVER_URL}", file=sys.stderr)
    print("=" * 60, file=sys.stderr)
    
    if not OPENAI_API_KEY:
        print("\nADVERTENCIA: OPENAI_API_KEY no configurada", file=sys.stderr)
    else:
        print(f"OpenAI API Key configurada", file=sys.stderr)
    
    print("\nIMPORTANTE: Asegúrate de que mcp_server.py esté corriendo primero!", file=sys.stderr)
    print("Iniciando servidor...", file=sys.stderr)
    
    uvicorn.run(app, host=HOST, port=PORT, log_level="info")
