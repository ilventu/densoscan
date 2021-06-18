#ifndef OPTIONS_H
#define OPTIONS_H

#include <QDialog>

namespace Ui {
class Options;
}

class Options : public QDialog
{
    Q_OBJECT

public:
    explicit Options(QWidget *parent = nullptr);
    ~Options();

    void doModal ( int &previewDPI, bool &keepFrameBorders, int &scanner_debug );

private:
    Ui::Options *ui;
};

#endif // OPTIONS_H
