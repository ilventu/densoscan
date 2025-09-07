#ifndef DENSOSCAN_WX_H
#define DENSOSCAN_WX_H


#include <wx/wx.h>
#include <wx/frame.h>
#include <wx/spinctrl.h>

// Forward declarations for custom controls and logic
// class Scanner;
// class Profile;
// ...



class DensoScanFrame : public wxFrame {
public:
    DensoScanFrame(const wxString& title);
    ~DensoScanFrame();

private:
    // Controlli principali
    wxPanel* mainPanel;
    wxBoxSizer* mainSizer;
    wxBoxSizer* leftSizer;
    wxBoxSizer* rightSizer;

    // Controlli lato sinistro (in ordine di UI)
    wxComboBox* comboDevice;
    wxButton* buttonRefresh;
    wxComboBox* comboDPI;
    wxComboBox* comboType;
    wxComboBox* comboDetectionMode;
    wxComboBox* outputType;
    wxComboBox* comboProfile;
    wxComboBox* comboOutputDPI;
    wxComboBox* comboInterpolation;
    wxComboBox* startingIndex;
    wxComboBox* comboBatch;
    wxCheckBox* checkBatch;
    wxSpinCtrl* brightness;
    wxTextCtrl* folderName;
    wxButton* toolSelectFolder;
    wxTextCtrl* rollName;
    wxButton* buttonOptions;
    wxButton* buttonScan;
    wxButton* buttonCancel;
    wxGauge* progressBar;

    // Etichette
    wxStaticText* labelDPI;
    wxStaticText* labelProfile;
    wxStaticText* labelOutputType;
    wxStaticText* labelFilmType;
    wxStaticText* labelExposureComp;
    wxStaticText* labelOutputFolder;
    wxStaticText* labelOutputDPI;
    wxStaticText* labelInterpolation;
    wxStaticText* labelFramesDetection;
    wxStaticText* labelRollName;
    wxStaticText* labelFirstFrame;
    wxStaticText* labelBatch;

    // Area anteprima
    wxStaticBitmap* preview;

    // Event handlers
    void OnScan(wxCommandEvent& event);
    void OnCancel(wxCommandEvent& event);
    void OnOptions(wxCommandEvent& event);
    void OnRefresh(wxCommandEvent& event);
    void OnSelectFolder(wxCommandEvent& event);

    wxDECLARE_EVENT_TABLE();
};

#endif // DENSOSCAN_WX_H
