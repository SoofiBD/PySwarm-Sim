from PySide6.QtGui import QValidator
from typing import Tuple

class IPv4Validator(QValidator):
    def validate(self, string: str, index: int) -> Tuple[QValidator.State, str, int]:
        octets = string.split('.')
        if len(octets) > 4:
            state = QValidator.State.Invalid
        elif not all([x.isdigit() for x in octets if x != '']):
            state = QValidator.State.Invalid
        elif not all([0 <= int(x) <= 255 for x in octets if x != '']):
            state = QValidator.State.Invalid
        elif len(octets) < 4:
            state = QValidator.State.Intermediate
        elif any([x == '' for x in octets]):
            state = QValidator.State.Intermediate
        else:
            state = QValidator.State.Acceptable
        return (state, string, index)

class XYZValidator(QValidator):
    def validate(self, string: str, index: int) -> Tuple[QValidator.State, str, int]:
        if string == "" or string == "." or string == "-":
            return (QValidator.State.Intermediate, string, index)
        
        try:
            val = float(string)
            if 0 <= val <= 800:
                return (QValidator.State.Acceptable, string, index)
            else:
                return (QValidator.State.Invalid, string, index)
        except ValueError:
            return (QValidator.State.Invalid, string, index)

class numValidator(QValidator):
    def validate(self, string: str, index: int) -> Tuple[QValidator.State, str, int]:
        if string == "":
            return (QValidator.State.Intermediate, string, index)
        try:
            val = float(string)
            if 1 <= val <= 50: # Increased limit slightly for flexibility
                return (QValidator.State.Acceptable, string, index)
            else:
                return (QValidator.State.Invalid, string, index)
        except ValueError:
            return (QValidator.State.Invalid, string, index)
