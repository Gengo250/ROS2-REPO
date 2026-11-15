"""Repository locations used by the vision tools (the workspace lives in <repo>/vision)."""
import glob
import os
from pathlib import Path

VISION = Path(__file__).resolve().parents[1]
REPO = VISION.parent
WEB = REPO/'web/digital-twin'
WEB_DIST = WEB/'dist'
SCENE = REPO/'src/atlas_scene/config/atlas_campus.scene.json'
CAMERAS = WEB/'public/robots/amr.cameras.json'


def chromium():
    """Chromium for the render worker: $ATLAS_CHROMIUM, else Playwright's build used by the web tests."""
    explicit = os.environ.get('ATLAS_CHROMIUM')
    if explicit:
        return explicit
    found = sorted(glob.glob(str(Path.home()/'.cache/ms-playwright/chromium-*/chrome-linux64/chrome')))
    if not found:
        raise FileNotFoundError('No Chromium found: set ATLAS_CHROMIUM or run `npx playwright install chromium` in web/digital-twin')
    return found[-1]
