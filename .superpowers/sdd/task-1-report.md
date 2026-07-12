## Task 1 Report: Raw Image Encoding Dispatch Table

**Status:** DONE

**Commits:**
- `b13d75f` feat: add encoding dispatch table for mono8/bgr8/rgb8/YUV formats

**Test summary:**
- Syntax verified via `python3 -m py_compile scripts/validate_topics.py` -- compiles cleanly with no errors.
- No existing test suite for this script; the task only required syntax verification.

**What was implemented:**
Replaced the mono8-only decode logic in `_on_image` (lines 68-78) with an encoding dispatch table supporting 5 raw encodings:

| Encoding | Reshape | Conversion |
|---|---|---|
| `mono8` | `(H, W)` | none |
| `bgr8` | `(H, W, 3)` | none (OpenCV native) |
| `rgb8` | `(H, W, 3)` | `cv2.COLOR_RGB2BGR` |
| `yuv422_yuy2` | `(H, W, 2)` | `cv2.COLOR_YUV2BGR_YUYV` |
| `yuv422` | `(H, W, 2)` | `cv2.COLOR_YUV2BGR_UYVY` |
| unknown fallback | `(H, W)` | mono8 interpreted, with warning |

Minor improvements beyond spec: extracted `encoding` local variable, wrapped the encoding string in quotes in the warning message for readability.

**Concerns:**
- None.
