from __future__ import annotations

import importlib.util
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[1]


def _load():
    spec = importlib.util.spec_from_file_location(
        "markdown_dependencies", ROOT / "scripts/install_python_dependencies.py",
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_reads_only_dependency_block_and_resolves_relative_include(tmp_path):
    module = _load()
    (tmp_path / "base.md").write_text(
        "# 文档\n```requirements\nnumpy==1.26.4\n```\n", encoding="utf-8",
    )
    document = tmp_path / "rl.md"
    document.write_text(
        "说明不是依赖。\n```bash\necho ignored\n```\n"
        "```requirements\n# 依赖\n-r base.md\n"
        "--extra-index-url https://example.com/wheels\n"
        "torch==2.5.1+cu121\n```\n后续说明。\n", encoding="utf-8",
    )
    assert module.read_requirements(document) == [
        "numpy==1.26.4", "--extra-index-url", "https://example.com/wheels",
        "torch==2.5.1+cu121",
    ]


@pytest.mark.parametrize("text", [
    "只有正文", "```requirements\nnumpy==1.26.4\n",
    "```requirements\na==1\n```\n```requirements\nb==2\n```\n",
])
def test_invalid_document_is_rejected_before_install(tmp_path, text):
    module = _load()
    document = tmp_path / "invalid.md"
    document.write_text(text, encoding="utf-8")
    with pytest.raises(ValueError, match="requirements"):
        module.read_requirements(document)


def test_circular_include_is_rejected(tmp_path):
    module = _load()
    document = tmp_path / "cycle.md"
    document.write_text("```requirements\n-r cycle.md\n```\n", encoding="utf-8")
    with pytest.raises(ValueError, match="循环"):
        module.read_requirements(document)


def test_install_uses_requested_interpreter_and_preserves_pip_options(monkeypatch):
    module = _load()
    calls = []

    def run(command, check):
        calls.append(command)
        return type("Result", (), {"returncode": 7})()

    monkeypatch.setattr(module.subprocess, "run", run)
    result = module.main([
        "tensorrt", "--python", "/tmp/selected venv/bin/python", "--no-deps",
    ])
    assert result == 7
    assert calls == [[
        "/tmp/selected venv/bin/python", "-m", "pip", "install", "--no-deps",
        "tensorrt-cu12==10.13.3.9.post1",
        "tensorrt-cu12-bindings==10.13.3.9.post1",
        "tensorrt-cu12-libs==10.13.3.9.post1",
    ]]


def test_show_does_not_start_installation(monkeypatch, capsys):
    module = _load()
    monkeypatch.setattr(module.subprocess, "run", lambda *a, **kw: pytest.fail("installed"))
    assert module.main(["rl", "--show"]) == 0
    output = capsys.readouterr().out
    assert "mujoco==3.3.0" in output
    assert "torch==2.5.1+cu121" in output
    assert "--extra-index-url https://download.pytorch.org/whl/cu121" in output
