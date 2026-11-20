"""Update the design-analysis card while retaining reference replay and narration."""

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
import imageio_ffmpeg
from segd.final_design import calculate
from segd.technical_film import analysis_frame


def revise(source, out):
    source = Path(source).resolve()
    out = Path(out).resolve()
    if source == out:
        raise ValueError("Output must differ from source")
    ff = imageio_ffmpeg.get_ffmpeg_exe()
    probe = subprocess.run(
        [ff, "-hide_banner", "-i", str(source)], capture_output=True, text=True
    ).stderr
    if "1920x1080" not in probe or "00:02:16.00" not in probe:
        raise ValueError("Expected the released 136 s 1080p film")
    design = calculate()
    card = out.with_suffix(".analysis.png")
    analysis_frame(3, design).save(card)
    cmd = [
        ff,
        "-y",
        "-hide_banner",
        "-i",
        str(source),
        "-loop",
        "1",
        "-i",
        str(card),
        "-filter_complex",
        "[0:v][1:v]overlay=0:0:enable='gte(t,48)*lt(t,56)'[v]",
        "-map",
        "[v]",
        "-map",
        "0:a",
        "-map",
        "0:s?",
        "-t",
        "136",
        "-c:v",
        "libx264",
        "-crf",
        "18",
        "-preset",
        "fast",
        "-threads",
        "4",
        "-pix_fmt",
        "yuv420p",
        "-r",
        "30",
        "-c:a",
        "copy",
        "-c:s",
        "copy",
        "-movflags",
        "+faststart",
        str(out),
    ]
    subprocess.run(cmd, check=True)

    def audio_hash(p):
        return subprocess.run(
            [
                ff,
                "-v",
                "error",
                "-i",
                str(p),
                "-map",
                "0:a",
                "-c",
                "copy",
                "-f",
                "hash",
                "-hash",
                "sha256",
                "-",
            ],
            capture_output=True,
            text=True,
            check=True,
        ).stdout.strip()

    before = audio_hash(source)
    after = audio_hash(out)
    assert before == after, "Narration packet data changed"
    record = {
        "source_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "output_sha256": hashlib.sha256(out.read_bytes()).hexdigest(),
        "updated_interval_s": [48, 56],
        "updated_content": "100 x 44 mm short-code label; matched optical bound; 18 V pack; five turn events in mission budget",
        "audio_packets_unchanged": True,
        "audio_packet_sha256": after,
        "new_training_or_flight": False,
        "reference_replay_preserved_in_content": True,
        "original_youtube_upload_replaced": False,
        "adverse_wh": design["cases"][0]["adverse_wh"],
        "required_delivered_wh": design["cases"][0]["required_delivered_wh_adverse"],
    }
    out.with_suffix(".revision.json").write_text(json.dumps(record, indent=2) + "\n")


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--source", required=True)
    p.add_argument("--out", required=True)
    a = p.parse_args()
    revise(a.source, a.out)
