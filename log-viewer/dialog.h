#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include "logging.pb.h"
#include <vector>
#include <QTimer>

using namespace std;

namespace Ui {
class Dialog;
}

static const int timeMs = 16;   // for moving slider automatically
class Dialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit Dialog(QWidget *parent = 0);
    ~Dialog();
    
private slots:
    void on_loadButton_clicked();

    void onCurIdxChanged(int idx); // idx is index of data
    void onTimeout();
    void on_startButton_clicked();

    void on_pauseButton_clicked();

    void on_resetButton_clicked();

    void on_horizontalSlider_sliderMoved(int position);

    void on_saveAsButton_clicked();

private:
    // use ui->horizontalSlider-setValue() to get index in data.
    Ui::Dialog *ui;
    Logging::Log data;

    QTimer *timer;
};

#endif // DIALOG_H
