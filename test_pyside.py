import sys
from PySide6.QtWidgets import QApplication, QWidget

if __name__ == "__main__":
    app = QApplication(sys.argv)
    w = QWidget()
    w.setWindowTitle("Test")
    w.show()
    print("UI Started")
    # Don't app.exec() as it will hang in headless
    sys.exit(0)
