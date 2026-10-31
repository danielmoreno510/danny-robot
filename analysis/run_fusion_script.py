"""
Danny - Ejecuta en Fusion un script de cad/fusion, a través del complemento DannyBridge.

Fusion debe estar abierto y con el complemento activado. Muestra lo que devuelve el
script y deja la imagen de la vista en cad/fusion/bridge_view.png. Sale con código 1 si
el script falla o el puente no responde.

Uso: .venv/bin/python analysis/run_fusion_script.py ReportFusionState
"""

import argparse
import json
import os
import pathlib
import sys
import time
import uuid

CAD_DIR = pathlib.Path(__file__).resolve().parent.parent / "cad" / "fusion"
REQUEST = CAD_DIR / "bridge_request.json"
RESPONSE = CAD_DIR / "bridge_response.json"
POLL_SECONDS = 0.5


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("script", help="carpeta de cad/fusion, por ejemplo BuildExportTest")
    parser.add_argument("--timeout", type=float, default=120, help="espera máxima, en segundos")
    args = parser.parse_args()

    if not (CAD_DIR / args.script / f"{args.script}.py").is_file():
        print(f"No existe el script {args.script} en {CAD_DIR}")
        return 1

    request_id = uuid.uuid4().hex
    pending = REQUEST.with_suffix(".tmp")
    pending.write_text(json.dumps({"id": request_id, "script": args.script}), encoding="utf-8")
    os.replace(pending, REQUEST)

    deadline = time.monotonic() + args.timeout
    while time.monotonic() < deadline:
        if RESPONSE.exists():
            answer = json.loads(RESPONSE.read_text(encoding="utf-8"))
            if answer.get("id") == request_id:
                return report(answer)
        time.sleep(POLL_SECONDS)

    REQUEST.unlink(missing_ok=True)
    print("Fusion no ha respondido: comprueba que está abierto y que DannyBridge está activado.")
    return 1


def report(answer):
    if "error" in answer:
        print(answer["error"])
        return 1
    print(json.dumps(answer["result"], indent=1, ensure_ascii=False))
    print(f"Imagen de la vista guardada: {answer.get('image', False)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
