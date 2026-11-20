"""Authoritative XeLaTeX export with synchronized Markdown/Word editing copies."""

import json, re, os, subprocess, hashlib, sys, shutil
from pathlib import Path
from .flight import ROOT


def escape(text):
    return "".join(
        {
            "\\": r"\textbackslash{}",
            "&": r"\&",
            "%": r"\%",
            "$": r"\$",
            "#": r"\#",
            "_": r"\_",
            "{": r"\{",
            "}": r"\}",
            "~": r"\textasciitilde{}",
            "^": r"\textasciicircum{}",
        }.get(c, c)
        for c in text
    )


def prose(text):
    if "**" in text:
        parts = re.split(r"(\*\*.*?\*\*)", text)
        return "".join(
            r"\textbf{" + prose(p[2:-2]) + "}" if p.startswith("**") else prose(p)
            for p in parts
        )
    links = re.split(r"(\[[^\]]+\]\([^)]+\))", text)
    if len(links) > 1:
        parts = []
        for part in links:
            match = re.fullmatch(r"\[([^\]]+)\]\(([^)]+)\)", part)
            parts.append(
                r"\href{" + match[2] + "}{" + escape(match[1]) + "}"
                if match
                else prose(part)
            )
        return "".join(parts)
    if "$" in text:
        segments = re.split(r"(\$[^$]+\$)", text)
        if len(segments) == 1:
            raise ValueError("Unmatched inline-math delimiter")
        return "".join(
            part if part.startswith("$") and part.endswith("$") else prose(part)
            for part in segments
        )
    parts = re.split(r"(\[[0-9, –-]+\])", text)
    output = []
    for part in parts:
        if re.fullmatch(r"\[[0-9, –-]+\]", part):
            ids = []
            for item in part[1:-1].split(","):
                span = re.split("[–-]", item.strip())
                ids.extend(range(int(span[0]), int(span[-1]) + 1))
            output.append(r"\cite{" + ",".join("r" + str(i) for i in ids) + "}")
        else:
            output.append(escape(part))
    value = "".join(output)
    value = re.sub(r"Figure (\d+)", lambda m: r"Figure~\ref{fig:" + m[1] + "}", value)
    return value


def smallcaps(text, size=11.5):
    # Supplied Times has no smcp glyphs; keep its outlines for synthetic caps.
    return "".join(
        (
            r"{\fontsize{"
            + str(round(size * 0.8, 2))
            + "}{14}\selectfont "
            + escape(p.upper())
            + "}"
        )
        if p.islower()
        else escape(p)
        for p in re.split(r"([a-z]+)", text)
    )


def coloured_title(text):
    colours = {
        "Autonomous": "titleblue",
        "Warehouse": "titlegreen",
        "Inventory": "titlecoral",
        "Small": "titleorange",
        "UAV": "titleblue",
    }
    return " ".join(
        (
            r"\textcolor{"
            + colours[word]
            + "}{"
            + escape(word[0])
            + "}"
            + smallcaps(word[1:], 17.28)
        )
        if word in colours
        else smallcaps(word, 17.28)
        for word in text.split()
    )


def acronym_title():
    return "".join(
        r"\textcolor{" + colour + "}{" + letter + "}"
        for letter, colour in zip(
            "AWISU",
            ["titleblue", "titlegreen", "titlecoral", "titleorange", "titleblue"],
        )
    )


def typeset(content, sources):
    from .report import blocks

    meta = json.loads((ROOT / "docs/report_metadata.json").read_text())
    main = content.split("\n## References\n")[0]
    items = list(blocks(main))
    out = []
    table = 0
    pending_caption = None
    abstract_open = False
    for block in items:
        if block[0] == "heading":
            _, level, title = block
            title = re.sub(r"^\d+(?:\.\d+)*\.?\s+", "", title)
            if abstract_open:
                out.append(r"\end{abstract}")
                abstract_open = False
            if title == "Abstract":
                out.append(r"\begin{abstract}")
                abstract_open = True
                continue
            if level == 1:
                out.append(
                    r"\begin{center}\vspace*{6pt}{\fontsize{17.28}{20}\selectfont "
                    + acronym_title()
                    + ": "
                    + coloured_title("Autonomous Warehouse")
                    + r"\\"
                    + coloured_title("Inventory with a Small UAV")
                    + r"\par}\vspace{12pt}\textbf{"
                    + escape(meta["author"])
                    + r"}\\[8pt]\href{"
                    + meta["code"]
                    + r"}{\raisebox{-2pt}{\includegraphics[height=11.5pt]{figures/github.pdf}} Code}\quad\href{"
                    + meta["video"]
                    + r"}{\raisebox{-2pt}{\includegraphics[height=10pt]{figures/youtube.pdf}} YouTube}\end{center}"
                )
            else:
                out.append(
                    ("\\section" if level == 2 else "\\subsection")
                    + ("*" if title == "Abstract" else "")
                    + "{"
                    + smallcaps(title, 14 if level == 2 else 11.5)
                    + "}"
                )
        elif block[0] == "paragraph":
            value = block[1]
            if (
                value == meta["author"]
                or value.startswith("[Code](")
                or value.startswith("[Project code](")
            ):
                continue
            if value.startswith("EE5110/6110 Continuous Assessment"):
                out.append(r"\begin{center}" + escape(value) + r"\end{center}")
                continue
            if value.startswith("$$ "):
                assert value.endswith(" $$"), value
                # Keep the display in its lead-in paragraph to avoid extra parskip.
                if out:
                    out[-1] = out[-1].rstrip("\n")
                out.append(r"\begin{equation}" + value[3:-3] + r"\end{equation}")
            elif re.match(r"Table \d+\.", value):
                pending_caption = re.sub(r"^Table \d+\.\s*", "", value)
            else:
                out.append(prose(value) + "\n")
        elif block[0] == "figure":
            title, path = block[1:]
            number = re.match(r"Figure (\d+)\.", title)[1]
            caption = re.sub(r"^Figure \d+\.\s*", "", title)
            out.append(
                r"\par\addvspace{8pt}\noindent\begin{minipage}{\linewidth}\centering\includegraphics[width=\textwidth]{"
                + path
                + "}\n"
                + r"\captionof{figure}{"
                + prose(caption)
                + r"}\label{fig:"
                + number
                + r"}\end{minipage}\par\addvspace{8pt}"
            )
        else:
            rows = block[1]
            n = len(rows[0])
            table += 1
            weights = (
                ([0.44, 0.13, 0.43] if "Mass" in rows[0][1] else [0.29, 0.29, 0.42])
                if n == 3
                else [0.26, 0.25, 0.25, 0.24]
                if n == 4
                else [1 / n] * n
            )
            if rows[0][0] == "Measure":
                weights = [0.19, 0.19, 0.19, 0.43]
            # column spaces total 8pt per column; width also subtracts outer padding.
            numeric = [
                all(re.fullmatch(r"-?[0-9]+(?:\.[0-9]+)?", row[i]) for row in rows[1:])
                for i in range(n)
            ]
            widths = [
                f"C{{{w:.3f}\\dimexpr\\linewidth-{8 * n}pt\\relax}}" for w in weights
            ]
            short = len(rows) <= 9 or rows[0][0] == "Phase"
            spec = "".join(widths)
            if short:
                out.append(
                    r"\par\addvspace{8pt}\noindent\begin{minipage}{\linewidth}\centering"
                )
                if pending_caption:
                    out.append(
                        r"\captionof{table}{"
                        + prose(pending_caption)
                        + r"}\label{tab:"
                        + str(table)
                        + "}"
                    )
                    pending_caption = None
                out.append(r"\begin{tabular}{" + spec + "}")
            else:
                out.append(
                    r"\reportneedspace{8\baselineskip}\begin{longtable}{" + spec + "}"
                )
                if pending_caption:
                    out.append(
                        r"\caption{"
                        + prose(pending_caption)
                        + r"}\label{tab:"
                        + str(table)
                        + r"}\\"
                    )
                    pending_caption = None
            header = (
                " & ".join(r"\textbf{" + prose(v) + "}" for v in rows[0])
                + r"\\\midrule"
            )
            out.extend([r"\toprule", header])
            if not short:
                out.extend([r"\endfirsthead", r"\toprule", header, r"\endhead"])
            for row in rows[1:]:
                vals = [prose(v) for v in row]
                if row[0].startswith("Total") or row[-1] == "Selected":
                    vals = [r"\textbf{" + v + "}" for v in vals]
                out.append(" & ".join(vals) + r"\\")
            out.append(
                r"\bottomrule"
                + (
                    r"\end{tabular}\end{minipage}\par\addvspace{8pt}"
                    if short
                    else r"\end{longtable}"
                )
            )
    # Cite only sources actually used in narrative or figures, in numbered order.
    used = set(
        re.findall(r"r(\d+)", "".join(re.findall(r"\\cite\{([^}]+)\}", "\n".join(out))))
    )
    out.append(
        r"\renewcommand{\refname}{"
        + smallcaps("References", 14)
        + r"}\begin{thebibliography}{99}\interlinepenalty=10000\renewcommand{\makelabel}[1]{\hfill#1}\setlength{\itemsep}{1pt}\setlength{\parskip}{0pt}"
    )
    for s in sources:
        if str(s["id"]) not in used:
            continue
        title = escape(s.get("bibliography", s["title"]))
        url = s["url"]
        link = (
            r"\href{" + url + r"}{Source}"
            if url.startswith("http")
            else "Supplied course document"
        )
        out.append(
            r"\bibitem{r"
            + str(s["id"])
            + "} "
            + (
                r"\href{" + url + "}{" + title + "}"
                if url.startswith("http")
                else title
            )
            + "."
        )
    out.append(r"\end{thebibliography}")
    return "\n".join(out), len(used)


def word_copy(content, out):
    from .report import blocks
    from docx import Document
    from docx.shared import Pt, Mm

    doc = Document()
    sec = doc.sections[0]
    sec.page_width = Mm(210)
    sec.page_height = Mm(297)
    for name in [
        "Normal",
        "Title",
        "Heading 1",
        "Heading 2",
        "Heading 3",
        "Caption",
        "Footer",
    ]:
        style = doc.styles[name]
        style.font.name = "Times New Roman"
        style.font.size = Pt(
            20
            if name == "Title"
            else 14
            if name in ["Heading 1", "Heading 2"]
            else 11.5
        )
    for b in blocks(content):
        if b[0] == "heading":
            doc.add_heading(b[2], level=min(3, b[1]))
        elif b[0] == "paragraph":
            para = doc.add_paragraph()
            for part in re.split(r"(\*\*.*?\*\*)", b[1]):
                run = para.add_run(part[2:-2] if part.startswith("**") else part)
                run.bold = part.startswith("**")
        elif b[0] == "table":
            t = doc.add_table(rows=0, cols=len(b[1][0]))
            t.style = "Light Shading Accent 1"
            for values in b[1]:
                cells = t.add_row().cells
                for c, v in zip(cells, values):
                    c.text = v
        else:
            png = (out / b[2]).with_suffix(".png")
            doc.add_picture(str(png), width=Mm(170))
            doc.add_paragraph(b[1], style="Caption")
    doc.core_properties.title = "AWISU: Autonomous Warehouse Inventory with a Small UAV: System Design and Experimental Evaluation"
    doc.core_properties.author = "Junjie Peng"
    doc.save(out / "report.docx")


def _build_latex(results, out):
    from .report import resolved_source
    from .final_design import calculate
    from .final_figures import make_figures

    out.mkdir(parents=True, exist_ok=True)
    d = calculate()
    content, _, _ = resolved_source(results)
    figures = make_figures(results, out / "figures", d)
    envfig = os.environ.copy()
    envfig["SEGD_FIGURE_OUT"] = str(out / "figures")
    envfig["SEGD_FIGURES_ONLY"] = "1"
    subprocess.run(
        [sys.executable, str(ROOT / "scripts/render_method_figures.py")],
        env=envfig,
        check=True,
        cwd=ROOT,
    )
    import cairosvg

    cairosvg.svg2pdf(
        url=str(ROOT / "assets/report_sources/youtube.svg"),
        write_to=str(out / "figures/youtube.pdf"),
    )
    usedfigs = {
        "framework",
        "hardware",
        "04_observability",
        "05_imaging",
        "02_propulsion",
        "07_energy",
        "08_experiment",
    }
    figures = (
        [f for f in figures if f["figure"] in usedfigs]
        if figures and isinstance(figures[0], dict)
        else figures
    )
    manifest = json.loads((out / "figures/manifest.json").read_text())
    manifest["figures"] = [f for f in manifest["figures"] if f["figure"] in usedfigs]
    manifest["figures"][:0] = [
        {
            "figure": n,
            "formats": ["svg", "pdf", "png"],
            "source": "scripts/render_method_figures.py",
            "data": n + ".data.json",
        }
        for n in ["framework", "hardware"]
    ]
    (out / "figures/manifest.json").write_text(json.dumps(manifest, indent=2))
    figures = manifest["figures"]
    (out / "report.md").write_text(content)
    (out / "design_calculations.json").write_text(json.dumps(d, indent=2) + "\n")
    body, nrefs = typeset(content, json.loads((ROOT / "docs/sources.json").read_text()))
    source = (ROOT / "docs/latex/report.tex").read_text().replace("@@CONTENT@@", body)
    # Resolve font path for arbitrary output locations without copying licensed fonts.
    font_dir = Path(
        os.environ.get("AWISU_FONT_DIR", str(ROOT / "assets/fonts"))
    ).resolve()
    if not all(
        (font_dir / n).exists()
        for n in ["Times.TTF", "Timesbd.TTF", "Timesi.TTF", "Timesbi.TTF"]
    ):
        raise FileNotFoundError(
            "Set AWISU_FONT_DIR to a directory with your licensed Times New Roman fonts; see assets/fonts/README.md"
        )
    source = source.replace("Path=../assets/fonts/", "Path=" + str(font_dir) + "/")
    (out / "report.tex").write_text(source)
    word_copy(content, out)
    engine = os.environ.get("XELATEX") or shutil.which("xelatex")
    if not engine:
        raise FileNotFoundError("Install XeLaTeX or set XELATEX to its executable path")
    env = os.environ.copy()
    env["PATH"] = str(Path(engine).parent) + ":" + env["PATH"]
    for _ in range(3):
        proc = subprocess.run(
            [
                str(engine),
                "-no-shell-escape",
                "-interaction=nonstopmode",
                "-halt-on-error",
                "report.tex",
            ],
            cwd=out,
            env=env,
            capture_output=True,
            text=True,
        )
        (out / "compile_output.txt").write_text(proc.stdout + proc.stderr)
        if proc.returncode:
            raise RuntimeError(proc.stdout[-5000:])
    info = subprocess.check_output(["pdfinfo", str(out / "report.pdf")], text=True)
    pages = int(re.search(r"Pages:\s+(\d+)", info)[1])
    protected = json.loads((ROOT / "docs/artifacts.json").read_text())[
        "experiment_sha256"
    ]
    assert all(
        hashlib.sha256((ROOT / p).read_bytes()).hexdigest() == h
        for p, h in protected.items()
    )
    status = {
        "pages": pages,
        "page_limit": 20,
        "requested_pages": "at most 20",
        "individual_project": True,
        "font": "Times New Roman body/captions 11.5 pt; 17.28 pt small-cap title with matching AWISU initials; 14 pt section headings; Comic Neue figure labels; Times-style math",
        "figures": len(figures),
        "references": nrefs,
        "authority": "XeLaTeX",
        "protected_artifacts_unchanged": True,
        "battery_delivered_energy_qualification": False,
        "loaded_pack_screening": "18 V, digitised 23 C typical curves, explicit connection/capacity sensitivity",
        "matched_plant_and_optics_records": "results/candidate18",
        "unknown_layout_integrated_experiment": False,
        "source": "docs/report_source.md",
        "template": "docs/latex/report.tex",
        "calculation_source": "src/segd/final_design.py",
    }
    (out / "report_build.json").write_text(json.dumps(status, indent=2) + "\n")
    (out / "format_status.txt").write_text(json.dumps(status, indent=2) + "\n")
    assert pages <= 20, f"Report exceeds 20 pages including references: {pages}"
    assert not re.search(
        r"Overfull|undefined references|Missing character",
        (out / "report.log").read_text(),
    ), "Unresolved typesetting issue"
    return status


def build_latex(results, out):
    import tempfile, shutil

    # Compile off the shared filesystem, then replace complete deliverables.
    with tempfile.TemporaryDirectory(prefix="segd-report-", dir="/tmp") as stage:
        stage = Path(stage)
        status = _build_latex(results, stage)
        out.mkdir(parents=True, exist_ok=True)
        for source in stage.rglob("*"):
            if not source.is_file() or source.suffix in {
                ".aux",
                ".log",
                ".out",
                ".toc",
            }:
                continue
            target = out / source.relative_to(stage)
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_name(target.name + ".new")
            shutil.copyfile(source, temporary)
            os.replace(temporary, target)
    return status
