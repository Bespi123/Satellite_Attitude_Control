#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Get local Dir
    QDir currentDir;
    m_sLocalDir = currentDir.absolutePath();
    ui->label_dir->setText(m_sLocalDir);

    //Get avaiable ports and display in comboBoxPort
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
        ui->comboBox_Port->addItem(info.portName());

    //Initialize serial Port
    m_qSerial = new QSerialPort();

    //Connect signals and slots
    connect(ui->comboBox_Port, SIGNAL(currentTextChanged(const QString &)), this , SLOT(connectSerialPort(const QString &)));
}

MainWindow::~MainWindow(){
    m_qSerial ->close();
    delete ui;
}

bool MainWindow::init(const QString &portName){
    //Local variables
    bool Status;
    //Assign Serial Port
    m_qSerial -> setPortName(portName);
    m_qSerial -> setBaudRate(QSerialPort::Baud9600);
    m_qSerial -> setDataBits(QSerialPort::Data8);
    m_qSerial -> setParity(QSerialPort::NoParity);
    m_qSerial -> setStopBits(QSerialPort::OneStop);
    m_qSerial -> setFlowControl(QSerialPort::NoFlowControl);
    m_qSerial -> open(QIODevice::ReadWrite);
    if(m_qSerial -> isOpen() == true){
        Status=true;
        connect(m_qSerial, SIGNAL(readyRead()), this, SLOT(readHandler()));
    }else
        Status=false;
    return Status;
}

void MainWindow::connectSerialPort(const QString &CurrentPort){
    //Init Serial Conection
    if (CurrentPort != m_sSerialPort){
        m_qSerial ->close();
        m_sSerialPort = CurrentPort;
        if(init(m_sSerialPort))
             ui->label_Conect -> setText("Connected");
        else
            ui->label_Conect -> setText("Disconected");
    }
}

void MainWindow::readHandler(){
    //Local variables
    QByteArray rawData;

    //Copy incoming data and show it
    m_bReadData.append(m_qSerial->readAll());
    ui->textBrowser->setText(m_bReadData);
    QScrollBar *sb = ui->textBrowser->verticalScrollBar();
    sb->setValue(sb->maximum());

    // if recording botton was clicked
    if(m_bMode){
        if(!m_bReadData.isEmpty()){
            //Open file
            QString dataSerial = m_bReadData;
            m_fQfile = new  QFile(m_sFileName);
            if(m_fQfile->open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Append)){
        //if(m_fQfile->open(QIODevice::Append)){
            //QTextStream dataFile (m_fQfile);
            //dataFile << dataSerial<<endl;
                m_fQfile->write(m_bReadData,m_bReadData.size());
                ui->label_Recording->setText("Grabando");
            }else{
                ui->label_Recording->setText("Error");
            }
        }
    }
}

void MainWindow::on_pushButton_grabar_clicked(){
    m_bMode = true;
    m_sFileName = ui->lineEdit_Archive->text();
}

void MainWindow::on_pushButton_Detener_clicked(){
    m_bMode = false;
    m_fQfile->close();
    ui->label_Recording->setText("No Grabando");
}
