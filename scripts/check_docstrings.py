"""AST-based linter that checks every module, class, and function has a docstring.

Exit code 0 if all files are clean, 1 otherwise.
"""

from __future__ import annotations

import ast
import sys
from collections.abc import Sequence
from pathlib import Path


_REPO = Path(__file__).resolve().parent.parent
_EXCLUDE_DIRS = frozenset({".venv", "__pycache__", ".git", ".mypy_cache", ".pytest_cache", ".ruff_cache"})


def _walk_py() -> list[Path]:
    """Return all ``.py`` files under the repo root, excluding known cache and venv directories."""
    return [p for p in _REPO.rglob("*.py") if not any(d in p.parts for d in _EXCLUDE_DIRS)]


def _node_is_test_decorator(node: ast.AST) -> bool:
    """Return True if *node* is a ``@pytest.fixture`` or similar test decorator."""
    if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute):
        return node.func.attr == "fixture"
    return False


def _node_name(node: ast.AST) -> str:
    """Return the display name of an AST node for error messages.

    For functions and classes this is the declared name; otherwise
    it is the AST type name in angle brackets.
    """
    if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef, ast.ClassDef)):
        return node.name
    return f"<{type(node).__name__}>"


def _check_body(nodes: Sequence[ast.AST], parents: list[str], file: Path) -> list[str]:
    """Recursively check a list of AST body nodes for missing docstrings.

    Parameters
    ----------
    nodes : Sequence[ast.AST]
        Child nodes of a module, class, or compound statement.
    parents : list[str]
        Ancestor names used to build the dotted path in error messages.
    file : Path
        Source file being linted (for error-report location).

    Returns
    -------
    list[str]
        Human-readable error messages, one per missing docstring.
    """
    errors: list[str] = []
    for node in nodes:
        if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)):
            path = ".".join(parents + [node.name])
            # Skip @overload stubs
            if any(isinstance(d, ast.Name) and d.id == "overload" for d in node.decorator_list):
                continue
            if not ast.get_docstring(node):
                if node.name == "__init__":
                    errors.append(f"{file}: {path} — missing docstring")
                elif not node.name.startswith("__"):
                    errors.append(f"{file}: {path} — missing docstring (public)")
                else:
                    errors.append(f"{file}: {path} — missing docstring")
            for child in node.body:
                _check_body([child], parents + [node.name], file)  # nested defs
        elif isinstance(node, ast.ClassDef):
            path = ".".join(parents + [node.name])
            if not ast.get_docstring(node):
                errors.append(f"{file}: {path} — missing class docstring")
            _check_body(node.body, parents + [node.name], file)
        elif isinstance(node, ast.AsyncFor | ast.For | ast.While | ast.If | ast.Try):
            _check_body(node.body, parents, file)
            if hasattr(node, "orelse"):
                _check_body(node.orelse, parents, file)
            if hasattr(node, "handlers"):
                for h in node.handlers:
                    _check_body(h.body, parents, file)
                if hasattr(node, "finalbody"):
                    _check_body(node.finalbody, parents, file)
    return errors


def main() -> int:
    """Walk all Python source files and report any missing docstrings.

    Returns
    -------
    int
        0 if all files pass, 1 if any errors are found.
    """
    exit_code = 0
    for file in sorted(_walk_py()):
        try:
            tree = ast.parse(file.read_text(encoding="utf-8"), filename=str(file))
        except SyntaxError as exc:
            print(f"{file}: syntax error — {exc}", file=sys.stderr)
            exit_code = 1
            continue

        if not ast.get_docstring(tree):
            print(f"{file}: — missing module docstring")
            exit_code = 1

        errors = _check_body(tree.body, [], file)
        for err in errors:
            print(err, file=sys.stderr)
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
