import sys
import os
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QPlainTextEdit, QHBoxLayout, QVBoxLayout

class MyApp(QWidget):
    def __init__(self):
        super().__init__()
        self.window_width, self.window_height = 1200, 900
        self.setMinimumSize(self.window_width, self.window_height)

        self.setWindowTitle('Command Line App')
        layout = QVBoxLayout()
        self.setLayout(layout)

        self.editorCommand = QPlainTextEdit()
        layout.addWidget(self.editorCommand, 3)

        self.editorOutput = QPlainTextEdit()
        layout.addWidget(self.editorOutput, 7)

        buttonLayout = QHBoxLayout()
        layout.addLayout(buttonLayout)

        self.button_run = QPushButton('&Run Command', clicked=self.runCommand)
        buttonLayout.addWidget(self.button_run)

        self.button_clear = QPushButton('&Clear', clicked=lambda: self.editorOutput.clear())
        buttonLayout.addWidget(self.button_clear)

        self.editorCommand.insertPlainText('dir')

    def runCommand(self):
        command_line = self.editorCommand.toPlainText().strip()
        p = os.popen(command_line)
        if p:
            self.editorOutput.clear()
            output = p.read()
            self.editorOutput.insertPlainText(output)

if __name__ == '__main__':
    # don't auto scale when drag app to a different monitor.
    # QApplication.setAttribute(Qt.HighDpiScaleFactorRoundingPolicy.PassThrough)
    
    app = QApplication(sys.argv)
    app.setStyleSheet('''
        QWidget {
            font-size: 30px;
        }
    ''')
    
    myApp = MyApp()
    myApp.show()

    try:
        sys.exit(app.exec_())
    except SystemExit:
        print('Closing Window...')