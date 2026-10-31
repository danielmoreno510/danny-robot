"""
Danny - Complemento que deja a un agente usar Fusion sin tocar la pantalla.

Vigila cad/fusion/bridge_request.json, ejecuta en el hilo principal de Fusion el
build(app) de un script de cad/fusion, guarda una imagen de la vista y responde en
cad/fusion/bridge_response.json. Solo ejecuta carpetas de cad/fusion: nunca rutas
que vengan en la petición.

Petición: {"id": "1", "script": "BuildExportTest"}. Se envía con
analysis/run_fusion_script.py.
"""

import importlib.util
import json
import os
import re
import threading
import traceback

import adsk.core


CAD_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
REQUEST = os.path.join(CAD_DIR, "bridge_request.json")
RESPONSE = os.path.join(CAD_DIR, "bridge_response.json")
IMAGE = os.path.join(CAD_DIR, "bridge_view.png")
EVENT_ID = "DannyBridgeRun"
POLL_SECONDS = 1.0

handlers = []
stop_flag = threading.Event()


def run_script(app, name):
    if not re.fullmatch(r"[A-Za-z]+", name) or name == "DannyBridge":
        raise ValueError(f"script no permitido: {name}")
    path = os.path.join(CAD_DIR, name, f"{name}.py")
    specification = importlib.util.spec_from_file_location(f"danny_{name}", path)
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    return module.build(app)


class RunHandler(adsk.core.CustomEventHandler):
    def notify(self, args):
        app = adsk.core.Application.get()
        answer = {}
        try:
            with open(REQUEST, encoding="utf-8") as file:
                request = json.load(file)
            os.remove(REQUEST)
            answer["id"] = request.get("id")
            answer["result"] = run_script(app, request["script"])
            viewport = app.activeViewport
            if viewport:
                viewport.fit()
                answer["image"] = viewport.saveAsImageFile(IMAGE, 1400, 900)
        except Exception:
            answer["error"] = traceback.format_exc()
        pending = f"{RESPONSE}.tmp"
        with open(pending, "w", encoding="utf-8") as file:
            json.dump(answer, file, indent=1, ensure_ascii=False)
        os.replace(pending, RESPONSE)


def watch(app):
    while not stop_flag.wait(POLL_SECONDS):
        if os.path.exists(REQUEST):
            app.fireCustomEvent(EVENT_ID)
            stop_flag.wait(POLL_SECONDS)


def run(context):
    app = adsk.core.Application.get()
    event = app.registerCustomEvent(EVENT_ID)
    handler = RunHandler()
    event.add(handler)
    handlers.append(handler)
    stop_flag.clear()
    threading.Thread(target=watch, args=(app,), daemon=True).start()
    with open(RESPONSE, "w", encoding="utf-8") as file:
        json.dump({"ready": True, "fusion": app.version}, file)


def stop(context):
    stop_flag.set()
    adsk.core.Application.get().unregisterCustomEvent(EVENT_ID)
