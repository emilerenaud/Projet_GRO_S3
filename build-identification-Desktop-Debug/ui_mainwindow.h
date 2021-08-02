/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.12.8
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCharts>
#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTextBrowser>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QLabel *label_9;
    QLabel *label_10;
    QLabel *label_pathCSV;
    QPushButton *pulseButton;
    QChartView *graph;
    QLabel *label_2;
    QDoubleSpinBox *PWMBox;
    QLabel *label_4;
    QFrame *line_3;
    QSpinBox *DurationBox;
    QPushButton *pushButton_Params;
    QLabel *label;
    QPushButton *pushButton;
    QLineEdit *JsonKey;
    QLabel *label_3;
    QLabel *label_8;
    QCheckBox *checkBox;
    QLabel *label_5;
    QComboBox *comboBoxPort;
    QLabel *label_6;
    QFrame *line;
    QLabel *label_11;
    QTextBrowser *textBrowser;
    QLabel *label_7;
    QWidget *widget;
    QLineEdit *lineEdit_Thresh;
    QLineEdit *lineEdit_Kd;
    QLineEdit *lineEdit_Ki;
    QLineEdit *lineEdit_Kp;
    QLineEdit *lineEdit_DesVal;
    QPushButton *pushButton_reset;
    QTextBrowser *textBrowser_2;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(1004, 517);
        MainWindow->setAcceptDrops(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QString::fromUtf8("centralWidget"));
        label_9 = new QLabel(centralWidget);
        label_9->setObjectName(QString::fromUtf8("label_9"));
        label_9->setGeometry(QRect(745, 127, 61, 17));
        label_10 = new QLabel(centralWidget);
        label_10->setObjectName(QString::fromUtf8("label_10"));
        label_10->setGeometry(QRect(745, 159, 66, 17));
        label_pathCSV = new QLabel(centralWidget);
        label_pathCSV->setObjectName(QString::fromUtf8("label_pathCSV"));
        label_pathCSV->setGeometry(QRect(9, 69, 251, 19));
        label_pathCSV->setFrameShape(QFrame::StyledPanel);
        label_pathCSV->setTextFormat(Qt::AutoText);
        pulseButton = new QPushButton(centralWidget);
        pulseButton->setObjectName(QString::fromUtf8("pulseButton"));
        pulseButton->setGeometry(QRect(9, 164, 171, 25));
        graph = new QChartView(centralWidget);
        graph->setObjectName(QString::fromUtf8("graph"));
        graph->setGeometry(QRect(9, 307, 441, 201));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(9, 132, 75, 17));
        PWMBox = new QDoubleSpinBox(centralWidget);
        PWMBox->setObjectName(QString::fromUtf8("PWMBox"));
        PWMBox->setGeometry(QRect(120, 100, 65, 26));
        PWMBox->setMinimum(-1.000000000000000);
        PWMBox->setMaximum(1.000000000000000);
        PWMBox->setSingleStep(0.100000000000000);
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QString::fromUtf8("label_4"));
        label_4->setGeometry(QRect(300, 10, 191, 17));
        label_4->setAlignment(Qt::AlignLeading|Qt::AlignLeft|Qt::AlignVCenter);
        line_3 = new QFrame(centralWidget);
        line_3->setObjectName(QString::fromUtf8("line_3"));
        line_3->setGeometry(QRect(710, 0, 16, 291));
        line_3->setFrameShape(QFrame::VLine);
        line_3->setFrameShadow(QFrame::Sunken);
        DurationBox = new QSpinBox(centralWidget);
        DurationBox->setObjectName(QString::fromUtf8("DurationBox"));
        DurationBox->setGeometry(QRect(120, 130, 65, 26));
        DurationBox->setKeyboardTracking(false);
        DurationBox->setMinimum(0);
        DurationBox->setMaximum(5000);
        DurationBox->setSingleStep(25);
        pushButton_Params = new QPushButton(centralWidget);
        pushButton_Params->setObjectName(QString::fromUtf8("pushButton_Params"));
        pushButton_Params->setGeometry(QRect(730, 230, 261, 25));
        label = new QLabel(centralWidget);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(9, 100, 91, 17));
        pushButton = new QPushButton(centralWidget);
        pushButton->setObjectName(QString::fromUtf8("pushButton"));
        pushButton->setGeometry(QRect(730, 260, 261, 25));
        JsonKey = new QLineEdit(centralWidget);
        JsonKey->setObjectName(QString::fromUtf8("JsonKey"));
        JsonKey->setGeometry(QRect(131, 268, 142, 25));
        JsonKey->setAlignment(Qt::AlignCenter);
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QString::fromUtf8("label_3"));
        label_3->setGeometry(QRect(10, 270, 112, 17));
        label_8 = new QLabel(centralWidget);
        label_8->setObjectName(QString::fromUtf8("label_8"));
        label_8->setGeometry(QRect(745, 95, 66, 17));
        checkBox = new QCheckBox(centralWidget);
        checkBox->setObjectName(QString::fromUtf8("checkBox"));
        checkBox->setGeometry(QRect(9, 40, 259, 23));
        label_5 = new QLabel(centralWidget);
        label_5->setObjectName(QString::fromUtf8("label_5"));
        label_5->setGeometry(QRect(9, 9, 101, 17));
        comboBoxPort = new QComboBox(centralWidget);
        comboBoxPort->setObjectName(QString::fromUtf8("comboBoxPort"));
        comboBoxPort->setGeometry(QRect(70, 10, 191, 25));
        label_6 = new QLabel(centralWidget);
        label_6->setObjectName(QString::fromUtf8("label_6"));
        label_6->setGeometry(QRect(750, 10, 78, 17));
        label_6->setAlignment(Qt::AlignCenter);
        line = new QFrame(centralWidget);
        line->setObjectName(QString::fromUtf8("line"));
        line->setGeometry(QRect(470, 320, 511, 16));
        line->setFrameShadow(QFrame::Plain);
        line->setLineWidth(3);
        line->setFrameShape(QFrame::HLine);
        label_11 = new QLabel(centralWidget);
        label_11->setObjectName(QString::fromUtf8("label_11"));
        label_11->setGeometry(QRect(745, 190, 33, 17));
        textBrowser = new QTextBrowser(centralWidget);
        textBrowser->setObjectName(QString::fromUtf8("textBrowser"));
        textBrowser->setGeometry(QRect(293, 40, 421, 251));
        QFont font;
        font.setPointSize(9);
        textBrowser->setFont(font);
        label_7 = new QLabel(centralWidget);
        label_7->setObjectName(QString::fromUtf8("label_7"));
        label_7->setGeometry(QRect(745, 64, 79, 17));
        widget = new QWidget(centralWidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        widget->setGeometry(QRect(729, 373, 79, 16));
        lineEdit_Thresh = new QLineEdit(centralWidget);
        lineEdit_Thresh->setObjectName(QString::fromUtf8("lineEdit_Thresh"));
        lineEdit_Thresh->setGeometry(QRect(830, 190, 142, 25));
        lineEdit_Kd = new QLineEdit(centralWidget);
        lineEdit_Kd->setObjectName(QString::fromUtf8("lineEdit_Kd"));
        lineEdit_Kd->setGeometry(QRect(830, 159, 142, 25));
        lineEdit_Ki = new QLineEdit(centralWidget);
        lineEdit_Ki->setObjectName(QString::fromUtf8("lineEdit_Ki"));
        lineEdit_Ki->setGeometry(QRect(830, 127, 142, 25));
        lineEdit_Kp = new QLineEdit(centralWidget);
        lineEdit_Kp->setObjectName(QString::fromUtf8("lineEdit_Kp"));
        lineEdit_Kp->setGeometry(QRect(830, 95, 142, 25));
        lineEdit_DesVal = new QLineEdit(centralWidget);
        lineEdit_DesVal->setObjectName(QString::fromUtf8("lineEdit_DesVal"));
        lineEdit_DesVal->setGeometry(QRect(830, 64, 142, 25));
        pushButton_reset = new QPushButton(centralWidget);
        pushButton_reset->setObjectName(QString::fromUtf8("pushButton_reset"));
        pushButton_reset->setGeometry(QRect(10, 239, 92, 25));
        textBrowser_2 = new QTextBrowser(centralWidget);
        textBrowser_2->setObjectName(QString::fromUtf8("textBrowser_2"));
        textBrowser_2->setEnabled(true);
        textBrowser_2->setGeometry(QRect(10, 200, 181, 21));
        textBrowser_2->setMaximumSize(QSize(16777215, 70));
        MainWindow->setCentralWidget(centralWidget);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "Interface Identification", nullptr));
        label_9->setText(QApplication::translate("MainWindow", "Valeur Ki", nullptr));
        label_10->setText(QApplication::translate("MainWindow", "Valeur Kd", nullptr));
        label_pathCSV->setText(QString());
        pulseButton->setText(QApplication::translate("MainWindow", "Commande de pulse", nullptr));
        label_2->setText(QApplication::translate("MainWindow", "Duree (ms)", nullptr));
        label_4->setText(QApplication::translate("MainWindow", "Messages Json de l'Arduino:", nullptr));
        pushButton_Params->setText(QApplication::translate("MainWindow", "Envoie Parametres", nullptr));
        label->setText(QApplication::translate("MainWindow", "Tension [-1,1]", nullptr));
        pushButton->setText(QApplication::translate("MainWindow", "Obtenir Parametres", nullptr));
        JsonKey->setText(QApplication::translate("MainWindow", "measurements", nullptr));
        label_3->setText(QApplication::translate("MainWindow", "Donnees brutes:", nullptr));
        label_8->setText(QApplication::translate("MainWindow", "Valeur Kp", nullptr));
        checkBox->setText(QApplication::translate("MainWindow", "Enregistrement des donnees sous:", nullptr));
        label_5->setText(QApplication::translate("MainWindow", "Port:", nullptr));
        label_6->setText(QApplication::translate("MainWindow", "Execice PID", nullptr));
        label_11->setText(QApplication::translate("MainWindow", "Seuil", nullptr));
        label_7->setText(QApplication::translate("MainWindow", "Val. desiree", nullptr));
        pushButton_reset->setText(QApplication::translate("MainWindow", "Reset Graph", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
