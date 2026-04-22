import json
import traceback

class Utils:

    @staticmethod
    def writeToFile(content: str, filePath: str) -> None:
        try:
            f = open(filePath, "a")
            f.write(content)
        except Exception:
            print('>>> traceback <<<')
            traceback.print_exc()
            print('>>> end of traceback <<<')

    @staticmethod
    def readJSON(filePath: str) -> dict:
        try:
            with open(filePath) as f:
                return json.load(f)
        except Exception:
            print('>>> traceback <<<')
            traceback.print_exc()
            print('>>> end of traceback <<<')
