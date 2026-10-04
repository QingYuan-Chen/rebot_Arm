"""Explicitly regenerate the deterministic task catalogue; never run at training reset."""
import json
from rebotarm_drl.reach_model import CATALOGUE, generate_catalogue

if __name__ == "__main__":
    if CATALOGUE.exists():
        raise FileExistsError("Refusing to replace the versioned evaluation catalogue")
    CATALOGUE.write_text(json.dumps(generate_catalogue(), indent=2) + "\n")
    print(CATALOGUE)
