import json
import traceback

class Utils:

    @staticmethod
    def writeToFile(content, filePath):
        try:
            f = open(filePath, "a")
            f.write(content)
        except:
            print('>>> traceback <<<')
            traceback.print_exc()
            print('>>> end of traceback <<<')

    @staticmethod
    def readJSON(filePath):
        try:
            with open(filePath) as f:
                return json.load(f)
        except:
            print('>>> traceback <<<')
            traceback.print_exc()
            print('>>> end of traceback <<<')
