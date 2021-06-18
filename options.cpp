#include "options.h"
#include "ui_options.h"

#include "scanner.h"

Options::Options(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::Options)
{
    ui->setupUi(this);
    setModal(true);
}

Options::~Options()
{
    delete ui;
}

void Options::doModal(int &previewDPI, bool &keepFrameBorders, int &scanner_debug)
{
    ui->comboDPI->setCurrentText( QString::number ( previewDPI ) );
    ui->comboCanvas->setCurrentIndex( keepFrameBorders );

    ui->checkFrames->setChecked( scanner_debug & DEBUG_FRAMES );
    ui->checkHolder->setChecked( scanner_debug & DEBUG_HOLDER );
    ui->checkRawScan->setChecked( scanner_debug & DEBUG_RAWSCAN );
    ui->checkRawPreview->setChecked( scanner_debug & DEBUG_RAWPREVIEW );
    ui->checkPCrop->setChecked( scanner_debug & DEBUG_PROCESSCROP );

    if ( exec() )
    {
        previewDPI = ui->comboDPI->currentText().toInt();
        keepFrameBorders = ui->comboCanvas->currentIndex();

        scanner_debug = 0;
        scanner_debug |= ui->checkFrames->isChecked() ? DEBUG_FRAMES : 0;
        scanner_debug |= ui->checkHolder->isChecked() ? DEBUG_HOLDER : 0;
        scanner_debug |= ui->checkRawScan->isChecked() ? DEBUG_RAWSCAN : 0;
        scanner_debug |= ui->checkRawPreview->isChecked() ? DEBUG_RAWPREVIEW : 0;
        scanner_debug |= ui->checkPCrop->isChecked() ? DEBUG_PROCESSCROP : 0;
    }
}
