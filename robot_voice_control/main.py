#!/usr/bin/env python3
# OpenAI Realtime voice-controlled robot mock (print-only)

import base64, json, os, queue, sys, threading, time
from dataclasses import dataclass
from typing import Dict, Optional

import dotenv, pyaudio, websocket

dotenv.load_dotenv()
API_KEY = os.getenv("OPENAI_API_KEY")
MODEL   = "gpt-realtime"
WS_URL  = f"wss://api.openai.com/v1/realtime?model={MODEL}"

# --------------------------------------------------------------------------- #
# Mock robot API                                                              #
# --------------------------------------------------------------------------- #
class Robot:
    def move_forward(self, distance: int = 16):
        print(f"[ROBOT] move_forward({distance})");             return "ok"
    def move_backward(self, distance: int = 16):
        print(f"[ROBOT] move_backward({distance})");            return "ok"
    def turn_left(self, degrees: int = 90):
        print(f"[ROBOT] turn_left({degrees})");                 return "ok"
    def turn_right(self, degrees: int = 90):
        print(f"[ROBOT] turn_right({degrees})");                return "ok"
    def stop(self):
        print(f"[ROBOT] stop()");                               return "ok"
    def set_lights(self, color: str = "white", pattern: str = "solid"):
        print(f"[ROBOT] set_lights({color},{pattern})");        return "ok"
    def dance(self):
        print(f"[ROBOT] dance()");                              return "ok"

# --------------------------------------------------------------------------- #
# Realtime client                                                             #
# --------------------------------------------------------------------------- #
@dataclass
class AudioCfg:
    rate: int      = 24_000
    width: int     = 2          # 16-bit PCM
    channels: int  = 1
    chunk_ms: int  = 200
    @property
    def frames(self): return int(self.rate * self.chunk_ms / 1000)

class Client:
    def __init__(self, api_key: str):
        self.api_key   = api_key
        self.ws: Optional[websocket.WebSocketApp] = None
        self.robot     = Robot()
        self.calls: Dict[str, Dict] = {}
        self.audio_cfg = AudioCfg()
        self.pa        = pyaudio.PyAudio()
        self.spk       = None
        self.micq      = queue.Queue()

    # ---------- tool schema ------------------------------------------------- #
    @property
    def tools(self):
        int_param  = lambda dflt: {"type": "integer", "default": dflt}
        str_param  = lambda dflt="": {"type": "string", "default": dflt}
        return [
            {"type":"function","name":"move_forward","description":"Move forward",
             "parameters":{"type":"object","properties":{"distance":int_param(16)},
                           "required":[]}},
            {"type":"function","name":"move_backward","description":"Move backward",
             "parameters":{"type":"object","properties":{"distance":int_param(16)},
                           "required":[]}},
            {"type":"function","name":"turn_left","description":"Turn left",
             "parameters":{"type":"object","properties":{"degrees":int_param(90)},
                           "required":[]}},
            {"type":"function","name":"turn_right","description":"Turn right",
             "parameters":{"type":"object","properties":{"degrees":int_param(90)},
                           "required":[]}},
            {"type":"function","name":"stop","description":"Stop movement",
             "parameters":{"type":"object","properties":{},"required":[]}},
            {"type":"function","name":"set_lights","description":"Set LEDs",
             "parameters":{"type":"object",
                           "properties":{"color":str_param("white"),
                                         "pattern":str_param("solid")},
                           "required":[]}},
            {"type":"function","name":"dance","description":"Dance routine",
             "parameters":{"type":"object","properties":{},"required":[]}},
        ]

    # ---------- ws event handlers ------------------------------------------ #
    def _on_open(self, ws):
        cfg = {
            "type":"session.update",
            "session":{
                "type":"realtime",
                "model":MODEL,
                "output_modalities":["audio"],
                "audio":{
                    "input":{"format":{"type":"audio/pcm","rate":self.audio_cfg.rate},
                             "turn_detection":{"type":"semantic_vad"}},
                    "output":{"format":{"type":"audio/pcm","rate":self.audio_cfg.rate},
                              "voice":"marin"}},
                "instructions":(
                    "# Role & Objective\n"
                    "Voice interface to control a mobile robot.\n"
                    "# Language\n"
                    "English only.\n"
                    "# Tools\n"
                    "- Call a movement function IMMEDIATELY after the user issues a command.\n"
                    "- Otherwise ask a concise clarification.\n"
                    "# Voice rules\n"
                    "- ≤2 sentences per reply."
                ),
                "tools":self.tools,
                "tool_choice":"auto"
            }
        }
        ws.send(json.dumps(cfg))
        self._start_audio_threads()

    def _on_message(self, ws, message: str):
        evt = json.loads(message)
        t   = evt.get("type","")
        if t=="response.output_audio.delta":
            self._play(evt["delta"]); return
        if t=="response.function_call_arguments.delta":
            cid = evt["call_id"]; self.calls.setdefault(cid,{"name":evt["name"],"args":""})
            self.calls[cid]["args"] += evt.get("delta",""); return
        if t=="response.function_call_arguments.done":
            call = self.calls.pop(evt["call_id"])
            args = json.loads(call["args"] or "{}")
            result = getattr(self.robot, call["name"])(**args)
            ws.send(json.dumps({"type":"conversation.item.create",
                                "item":{"type":"function_call_output",
                                        "call_id":evt["call_id"],
                                        "output":json.dumps(result)}}))
            return
        if t=="error": print("API-error",evt)

    def _on_close(self, *_): 
        if self.spk: self.spk.close()
        self.pa.terminate()
        sys.exit(0)

    def _on_error(self, _, err): print("WS-error",err,file=sys.stderr)

    # ---------- audio ------------------------------------------------------- #
    def _start_audio_threads(self):
        self.spk = self.pa.open(format=self.pa.get_format_from_width(self.audio_cfg.width),
                                channels=self.audio_cfg.channels, rate=self.audio_cfg.rate,
                                output=True, frames_per_buffer=self.audio_cfg.frames)
        threading.Thread(target=self._mic_reader,  daemon=True).start()
        threading.Thread(target=self._mic_sender,  daemon=True).start()

    def _mic_reader(self):
        mic = self.pa.open(format=self.pa.get_format_from_width(self.audio_cfg.width),
                           channels=self.audio_cfg.channels, rate=self.audio_cfg.rate,
                           input=True, frames_per_buffer=self.audio_cfg.frames)
        while True:
            self.micq.put(mic.read(self.audio_cfg.frames, exception_on_overflow=False))

    def _mic_sender(self):
        while True:
            data = self.micq.get();  b64 = base64.b64encode(data).decode()
            self.ws.send(json.dumps({"type":"input_audio_buffer.append","audio":b64}))

    def _play(self, b64_audio: str):
        self.spk.write(base64.b64decode(b64_audio))

    # ---------- public ------------------------------------------------------ #
    def run(self):
        hdr = {"Authorization":f"Bearer {self.api_key}"}
        self.ws = websocket.WebSocketApp(WS_URL, header=hdr,
                                         on_open=self._on_open,
                                         on_message=self._on_message,
                                         on_close=self._on_close,
                                         on_error=self._on_error)
        self.ws.run_forever()

# --------------------------------------------------------------------------- #
# entry                                                                       #
# --------------------------------------------------------------------------- #
if __name__ == "__main__":
    if not API_KEY:
        print("Missing OPENAI_API_KEY in environment", file=sys.stderr); sys.exit(1)
    Client(API_KEY).run()
