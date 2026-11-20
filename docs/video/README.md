# Technical video

Public link: https://youtu.be/NSdeKHdK3a8

Local final file: `../../submission/EE5110_A0354063J_JunjiePeng_Video.mp4`.

The final film is 136 seconds, 1920×1080 at 30 fps, with locally synthesized English narration. Voice: **Jenny (Dioco)**, synthesized using [Piper](https://github.com/OHF-Voice/piper1-gpl). The voice model was [en_GB-jenny_dioco-medium](https://huggingface.co/rhasspy/piper-voices/tree/main/en/en_GB/jenny_dioco/medium); model weights are not redistributed here.

| Time | Content |
| --- | --- |
| 0:00 | Reference aircraft rotation and proposed component choices |
| 0:24 | Proposed navigation, bilateral scanning and local recovery |
| 0:48 | Imaging and resource analysis |
| 0:56 | Retained continuous-flight replay with four synchronized views |
| 2:08 | Tracking comparison and saved-image decoding evidence |

`script.json` is the timed authored narration, `narration.srt` is the English subtitle file, and `transcript.txt` is the readable narration. `segments.json`, `original_provenance.json` and `original_validation.json` retain the original production record. Historical paths in these records identify original inputs, some of which were intentionally excluded from the cleaned release. Current retained-file checks are in `../artifacts.json`.

Draft edits, video segments, synthetic voice model files, intermediate audio and rendering screenshots are excluded. `segd.technical_film` retains the final visual renderer for new renders from the preserved poses. The original English audio is embedded in the final video and can be extracted with FFmpeg if required.
