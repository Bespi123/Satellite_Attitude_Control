#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtSerialPort/QSerialPort>
#include <QSerialPortInfo>
#include <QTextStream>
#include <QDebug>
#include <QFile>
#include <QDir>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

public:
    QSerialPort* m_qSerial;
    QByteArray m_bReadData;

public slots:
    void readHandler();
    void connectSerialPort(const QString &CurrentPort);

private slots:
    void on_pushButton_grabar_clicked();

    void on_pushButton_Detener_clicked();

private:
    bool init(const QString &portName);

private:
    Ui::MainWindow *ui;

private:
    QString m_sSerialPort;
    QString m_sLocalDir;
    QFile* m_fQfile;
    bool m_bMode = false;
};
#endif // MAINWINDOW_H
