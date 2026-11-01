"""Identify USB hardware that contradicts the selected car profile."""

from pathlib import Path

USB_DEVICES_ROOT = Path('/sys/bus/usb/devices')


class CarHardwareError(RuntimeError):
    """Detected hardware belongs to another car."""


def usb_ids_present(root: Path | None = None) -> set[str]:
    root = Path(root) if root is not None else USB_DEVICES_ROOT
    try:
        devices = list(root.iterdir())
    except FileNotFoundError:
        return set()
    ids = set()
    for device in devices:
        try:
            vendor = (device / 'idVendor').read_text().strip().lower()
            product = (device / 'idProduct').read_text().strip().lower()
        except (FileNotFoundError, NotADirectoryError):
            continue
        ids.add(f'{vendor}:{product}')
    return ids


def contradiction(profile, all_profiles, present: set[str]) -> str | None:
    own = set(profile.distinctive_usb_ids)
    for other in sorted(all_profiles, key=lambda car: car.id):
        if other.id != profile.id:
            for usb_id in sorted((set(other.distinctive_usb_ids) - own) & present):
                return f'USB device {usb_id} belongs to {other.id}, not {profile.id}.'
    return None
