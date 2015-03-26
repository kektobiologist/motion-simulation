/********************************************************************************
** Form generated from reading UI file 'dialog.ui'
**
** Created: Fri Oct 31 07:16:51 2014
**      by: Qt User Interface Compiler version 4.8.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_DIALOG_H
#define UI_DIALOG_H

#include <QtCore/QVariant>
#include <QtGui/QAction>
#include <QtGui/QApplication>
#include <QtGui/QButtonGroup>
#include <QtGui/QDialog>
#include <QtGui/QFormLayout>
#include <QtGui/QGroupBox>
#include <QtGui/QHBoxLayout>
#include <QtGui/QHeaderView>
#include <QtGui/QLabel>
#include <QtGui/QLineEdit>
#include <QtGui/QPushButton>
#include <QtGui/QSlider>
#include <QtGui/QSpacerItem>
#include <QtGui/QVBoxLayout>
#include <firarenderarea2.h>

QT_BEGIN_NAMESPACE

class Ui_Dialog
{
public:
    QHBoxLayout *horizontalLayout_2;
    QVBoxLayout *verticalLayout;
    FIRARenderArea2 *renderArea;
    QSlider *horizontalSlider;
    QHBoxLayout *horizontalLayout;
    QPushButton *startButton;
    QPushButton *pauseButton;
    QPushButton *resetButton;
    QSpacerItem *horizontalSpacer;
    QVBoxLayout *verticalLayout_3;
    QVBoxLayout *verticalLayout_2;
    QGroupBox *groupBox_2;
    QFormLayout *formLayout;
    QLabel *label;
    QLabel *sentVLabel;
    QLabel *label_2;
    QLabel *visionVLabel;
    QLabel *label_6;
    QLabel *tsSystemLabel;
    QGroupBox *groupBox;
    QFormLayout *formLayout_2;
    QLabel *label_3;
    QLabel *targetVLabel;
    QLabel *label_4;
    QLabel *ticksVLabel;
    QLabel *label_5;
    QLabel *tsReceivedLabel;
    QPushButton *saveAsButton;
    QLineEdit *lineEdit;
    QPushButton *loadButton;

    void setupUi(QDialog *Dialog)
    {
        if (Dialog->objectName().isEmpty())
            Dialog->setObjectName(QString::fromUtf8("Dialog"));
        Dialog->resize(782, 524);
        horizontalLayout_2 = new QHBoxLayout(Dialog);
        horizontalLayout_2->setSpacing(6);
        horizontalLayout_2->setContentsMargins(11, 11, 11, 11);
        horizontalLayout_2->setObjectName(QString::fromUtf8("horizontalLayout_2"));
        verticalLayout = new QVBoxLayout();
        verticalLayout->setSpacing(6);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        renderArea = new FIRARenderArea2(Dialog);
        renderArea->setObjectName(QString::fromUtf8("renderArea"));
        renderArea->setMinimumSize(QSize(605, 410));
        renderArea->setMaximumSize(QSize(605, 410));

        verticalLayout->addWidget(renderArea);

        horizontalSlider = new QSlider(Dialog);
        horizontalSlider->setObjectName(QString::fromUtf8("horizontalSlider"));
        horizontalSlider->setOrientation(Qt::Horizontal);

        verticalLayout->addWidget(horizontalSlider);

        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setSpacing(6);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        startButton = new QPushButton(Dialog);
        startButton->setObjectName(QString::fromUtf8("startButton"));

        horizontalLayout->addWidget(startButton);

        pauseButton = new QPushButton(Dialog);
        pauseButton->setObjectName(QString::fromUtf8("pauseButton"));

        horizontalLayout->addWidget(pauseButton);

        resetButton = new QPushButton(Dialog);
        resetButton->setObjectName(QString::fromUtf8("resetButton"));

        horizontalLayout->addWidget(resetButton);


        verticalLayout->addLayout(horizontalLayout);


        horizontalLayout_2->addLayout(verticalLayout);

        horizontalSpacer = new QSpacerItem(14, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout_2->addItem(horizontalSpacer);

        verticalLayout_3 = new QVBoxLayout();
        verticalLayout_3->setSpacing(6);
        verticalLayout_3->setObjectName(QString::fromUtf8("verticalLayout_3"));
        verticalLayout_2 = new QVBoxLayout();
        verticalLayout_2->setSpacing(6);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        groupBox_2 = new QGroupBox(Dialog);
        groupBox_2->setObjectName(QString::fromUtf8("groupBox_2"));
        formLayout = new QFormLayout(groupBox_2);
        formLayout->setSpacing(6);
        formLayout->setContentsMargins(11, 11, 11, 11);
        formLayout->setObjectName(QString::fromUtf8("formLayout"));
        label = new QLabel(groupBox_2);
        label->setObjectName(QString::fromUtf8("label"));

        formLayout->setWidget(0, QFormLayout::LabelRole, label);

        sentVLabel = new QLabel(groupBox_2);
        sentVLabel->setObjectName(QString::fromUtf8("sentVLabel"));

        formLayout->setWidget(0, QFormLayout::FieldRole, sentVLabel);

        label_2 = new QLabel(groupBox_2);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        formLayout->setWidget(1, QFormLayout::LabelRole, label_2);

        visionVLabel = new QLabel(groupBox_2);
        visionVLabel->setObjectName(QString::fromUtf8("visionVLabel"));

        formLayout->setWidget(1, QFormLayout::FieldRole, visionVLabel);

        label_6 = new QLabel(groupBox_2);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        formLayout->setWidget(2, QFormLayout::LabelRole, label_6);

        tsSystemLabel = new QLabel(groupBox_2);
        tsSystemLabel->setObjectName(QString::fromUtf8("tsSystemLabel"));

        formLayout->setWidget(2, QFormLayout::FieldRole, tsSystemLabel);


        verticalLayout_2->addWidget(groupBox_2);

        groupBox = new QGroupBox(Dialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        formLayout_2 = new QFormLayout(groupBox);
        formLayout_2->setSpacing(6);
        formLayout_2->setContentsMargins(11, 11, 11, 11);
        formLayout_2->setObjectName(QString::fromUtf8("formLayout_2"));
        label_3 = new QLabel(groupBox);
        label_3->setObjectName(QString::fromUtf8("label_3"));

        formLayout_2->setWidget(0, QFormLayout::LabelRole, label_3);

        targetVLabel = new QLabel(groupBox);
        targetVLabel->setObjectName(QString::fromUtf8("targetVLabel"));

        formLayout_2->setWidget(0, QFormLayout::FieldRole, targetVLabel);

        label_4 = new QLabel(groupBox);
        label_4->setObjectName(QString::fromUtf8("label_4"));

        formLayout_2->setWidget(1, QFormLayout::LabelRole, label_4);

        ticksVLabel = new QLabel(groupBox);
        ticksVLabel->setObjectName(QString::fromUtf8("ticksVLabel"));

        formLayout_2->setWidget(1, QFormLayout::FieldRole, ticksVLabel);

        label_5 = new QLabel(groupBox);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        formLayout_2->setWidget(2, QFormLayout::LabelRole, label_5);

        tsReceivedLabel = new QLabel(groupBox);
        tsReceivedLabel->setObjectName(QString::fromUtf8("tsReceivedLabel"));

        formLayout_2->setWidget(2, QFormLayout::FieldRole, tsReceivedLabel);


        verticalLayout_2->addWidget(groupBox);


        verticalLayout_3->addLayout(verticalLayout_2);

        saveAsButton = new QPushButton(Dialog);
        saveAsButton->setObjectName(QString::fromUtf8("saveAsButton"));

        verticalLayout_3->addWidget(saveAsButton);

        lineEdit = new QLineEdit(Dialog);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        verticalLayout_3->addWidget(lineEdit);

        loadButton = new QPushButton(Dialog);
        loadButton->setObjectName(QString::fromUtf8("loadButton"));

        verticalLayout_3->addWidget(loadButton);


        horizontalLayout_2->addLayout(verticalLayout_3);


        retranslateUi(Dialog);

        QMetaObject::connectSlotsByName(Dialog);
    } // setupUi

    void retranslateUi(QDialog *Dialog)
    {
        Dialog->setWindowTitle(QApplication::translate("Dialog", "log-viewer", 0, QApplication::UnicodeUTF8));
        startButton->setText(QApplication::translate("Dialog", "Start", 0, QApplication::UnicodeUTF8));
        pauseButton->setText(QApplication::translate("Dialog", "Pause", 0, QApplication::UnicodeUTF8));
        resetButton->setText(QApplication::translate("Dialog", "Reset", 0, QApplication::UnicodeUTF8));
        groupBox_2->setTitle(QApplication::translate("Dialog", "System", 0, QApplication::UnicodeUTF8));
        label->setText(QApplication::translate("Dialog", "Sent:", 0, QApplication::UnicodeUTF8));
        sentVLabel->setText(QString());
        label_2->setText(QApplication::translate("Dialog", "Vision:", 0, QApplication::UnicodeUTF8));
        visionVLabel->setText(QString());
        label_6->setText(QApplication::translate("Dialog", "Ts:", 0, QApplication::UnicodeUTF8));
        tsSystemLabel->setText(QString());
        groupBox->setTitle(QApplication::translate("Dialog", "Received", 0, QApplication::UnicodeUTF8));
        label_3->setText(QApplication::translate("Dialog", "Target:", 0, QApplication::UnicodeUTF8));
        targetVLabel->setText(QString());
        label_4->setText(QApplication::translate("Dialog", "Ticks:", 0, QApplication::UnicodeUTF8));
        ticksVLabel->setText(QString());
        label_5->setText(QApplication::translate("Dialog", "Ts:", 0, QApplication::UnicodeUTF8));
        tsReceivedLabel->setText(QString());
        saveAsButton->setText(QApplication::translate("Dialog", "Save Log As", 0, QApplication::UnicodeUTF8));
        lineEdit->setText(QApplication::translate("Dialog", "/home/robocup/motion-simulation/logs/1", 0, QApplication::UnicodeUTF8));
        loadButton->setText(QApplication::translate("Dialog", "Load Log File", 0, QApplication::UnicodeUTF8));
    } // retranslateUi

};

namespace Ui {
    class Dialog: public Ui_Dialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_DIALOG_H
