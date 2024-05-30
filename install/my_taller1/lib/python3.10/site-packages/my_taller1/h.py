import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QLabel
from PyQt5.uic import loadUi
import os
import threading


class MyWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        
        # Load the user interface from the .ui file
        loadUi('src/my_taller1/my_taller1/MainWindow.ui', self)
        
        # Set up event handlers
        #self.pushButton.clicked.connect(self.on_pushButton_clicked)

        self.button_run.clicked.connect(self.runCommand)
        self.button_clear.clicked.connect(self.editorOutput.clear)

        self.run_colconBuild.clicked.connect(self.colconbuild) 
        self.button_local_setup.clicked.connect(self.local_setup) 


        self.ros2_thread_position=threading.Thread(target=self.nodoPosition).start()
        self.nodo_Position.clicked.connect(self.nodoPosition)

        os.popen(". install/local_setup.sh")
        


    def colconbuild(self):
        #self.start_thread("ros2_commands")
        self.runRos2Command("colcon build")
        #self.end_thread("ros2_comands")


    def local_setup(self):
        #self.start_thread("ros2_commands")
        self.runRos2Command(". install/local_setup.sh")
        #self.end_thread("ros2_comands")


    def nodoPosition(self):
        try:
            p = os.popen("ros2 run my_taller1 nodo1")

            print("nodo1")
            self.position_status.setStyleSheet("background-color:rgb(86,250,29)")
            self.position_status.setText("Position:Connected")
            if p.read() =="Package 'my_taller1' not found":
                sys.exit()

            if p:
                self.editorOutput.clear()
                output = p.read()
                self.editorOutput.insertPlainText(output)
        except: 
            self.editorOutput.insertPlainText("Error ejecutando comando")

    
    def runCommand(self):
        
        command_line = self.editorCommand.toPlainText().strip()
        p = os.popen(command_line)
        try:
            if p:
                self.editorOutput.clear()
                output = p.read()
                self.editorOutput.insertPlainText(output)
        except:
            self.editorOutput.insertPlainText("Error ejecutando comando")



    def runRos2Command(self,ros2_message:str):

        
        #self.ros2_commands.start()
        command_line=""

        if ros2_message!=None:
            command_line = ros2_message
        
            self.editorCommand.clear()
            self.editorCommand.insertPlainText(command_line)
            
            p = os.popen(command_line)
            try:
                if p:
                    self.editorOutput.clear()
                    output = p.read()
                    self.editorOutput.insertPlainText(output)
            except:
                self.editorOutput.insertPlainText("Error ejecutando comando")
                #self.ros2_commands.quit()

            #self.ros2_commands.quit()

    def colconbuild(self):
        self.runRos2Command("colcon build")
    


    def local_setup(self):
        #self.start_thread("ros2_commands")
        self.runRos2Command(". install/local_setup.sh")
        #self.end_thread("ros2_comands")


if __name__ == '__main__':
    # Create a new application instance
    app = QApplication(sys.argv)

    # Create a new window instance
    window = MyWindow()

    # Show the window
    window.show()

    # Start the event loop
    sys.exit(app.exec_())

