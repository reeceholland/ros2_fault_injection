# Copyright 2026 Reece Holland
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.

import shutil
import subprocess
from pathlib import Path

project = "ros2_fault_injection"
author = "Reece Holland"
copyright = "2026, Reece Holland"

root_doc = "index"

extensions = [
    "breathe",
    "myst_parser",
]

templates_path = ["_templates"]
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

html_theme = "sphinx_rtd_theme"
html_static_path = ["_static"]

source_suffix = {
    ".rst": "restructuredtext",
    ".md": "markdown",
}

myst_enable_extensions = [
    "colon_fence",
    "deflist",
]

docs_dir = Path(__file__).resolve().parent
repo_root = docs_dir.parent
doxygen_output_dir = docs_dir / "_build" / "doxygen"
doxygen_xml_dir = doxygen_output_dir / "xml"


def run_doxygen() -> None:
    """Generate Doxygen XML for Breathe when Doxygen is available."""
    doxygen = shutil.which("doxygen")
    if doxygen is None:
        print("Doxygen not found; skipping C++ API XML generation.")
        return

    doxygen_output_dir.mkdir(parents=True, exist_ok=True)
    doxyfile = doxygen_output_dir / "Doxyfile"
    doxyfile.write_text(
        f"""
PROJECT_NAME           = "{project}"
PROJECT_BRIEF          = "ROS 2 fault injection framework"
OUTPUT_DIRECTORY       = "{doxygen_output_dir}"
CREATE_SUBDIRS         = NO

INPUT                  = "{repo_root / 'include'}" \
                         "{repo_root / 'src'}" \
                         "{repo_root / 'README.md'}" \
                         "{repo_root / 'docs' / 'mainpage.md'}"
FILE_PATTERNS          = *.hpp *.cpp *.md
RECURSIVE              = YES

USE_MDFILE_AS_MAINPAGE = "{repo_root / 'docs' / 'mainpage.md'}"
MARKDOWN_SUPPORT       = YES

EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = NO
EXTRACT_STATIC         = NO
HIDE_UNDOC_MEMBERS     = NO
HIDE_UNDOC_CLASSES     = NO
WARN_IF_UNDOCUMENTED   = NO
WARN_IF_DOC_ERROR      = YES

GENERATE_HTML          = NO
GENERATE_LATEX         = NO
GENERATE_XML           = YES
XML_OUTPUT             = xml

FULL_PATH_NAMES        = YES
STRIP_FROM_PATH        = "{repo_root}"
STRIP_FROM_INC_PATH    = "{repo_root / 'include'}"

JAVADOC_AUTOBRIEF      = YES
QT_AUTOBRIEF           = NO
MULTILINE_CPP_IS_BRIEF = NO

EXCLUDE_PATTERNS       = */build/* */install/* */log/* */.venv/* */docs/_build/*
""".strip()
        + "\n"
    )
    subprocess.run([doxygen, str(doxyfile)], check=True)


run_doxygen()

breathe_projects = {
    "ros2_fault_injection": str(doxygen_xml_dir),
}
breathe_default_project = "ros2_fault_injection"
