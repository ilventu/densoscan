#include "densoscan_wx.h"

// Costruttore: crea la finestra principale wxWidgets


// Costruttore: crea la finestra principale wxWidgets
DensoScanFrame::DensoScanFrame(const wxString& title)
    : wxFrame(nullptr, wxID_ANY, title, wxDefaultPosition, wxSize(960, 466))
{
    mainPanel = new wxPanel(this, wxID_ANY);
    mainSizer = new wxBoxSizer(wxHORIZONTAL);
    leftSizer = new wxBoxSizer(wxVERTICAL);
    rightSizer = new wxBoxSizer(wxVERTICAL);

    // --- Device row ---
    wxBoxSizer* deviceSizer = new wxBoxSizer(wxHORIZONTAL);
    labelDPI = new wxStaticText(mainPanel, wxID_ANY, "Device:");
    comboDevice = new wxComboBox(mainPanel, wxID_ANY);
    buttonRefresh = new wxButton(mainPanel, wxID_ANY, "⟳");
    deviceSizer->Add(labelDPI, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    deviceSizer->Add(comboDevice, 1, wxRIGHT, 5);
    deviceSizer->Add(buttonRefresh, 0);
    leftSizer->Add(deviceSizer, 0, wxEXPAND|wxALL, 5);

    // --- DPI row ---
    wxBoxSizer* dpiSizer = new wxBoxSizer(wxHORIZONTAL);
    labelDPI = new wxStaticText(mainPanel, wxID_ANY, "DPI:");
    comboDPI = new wxComboBox(mainPanel, wxID_ANY);
    dpiSizer->Add(labelDPI, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    dpiSizer->Add(comboDPI, 1);
    leftSizer->Add(dpiSizer, 0, wxEXPAND|wxALL, 5);

    // --- Film Type row ---
    wxBoxSizer* typeSizer = new wxBoxSizer(wxHORIZONTAL);
    labelFilmType = new wxStaticText(mainPanel, wxID_ANY, "Film Type:");
    comboType = new wxComboBox(mainPanel, wxID_ANY);
    typeSizer->Add(labelFilmType, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    typeSizer->Add(comboType, 1);
    leftSizer->Add(typeSizer, 0, wxEXPAND|wxALL, 5);

    // --- Frames detection row ---
    wxBoxSizer* detectionSizer = new wxBoxSizer(wxHORIZONTAL);
    labelFramesDetection = new wxStaticText(mainPanel, wxID_ANY, "Frames detection:");
    comboDetectionMode = new wxComboBox(mainPanel, wxID_ANY);
    comboDetectionMode->Append("Auto");
    comboDetectionMode->Append("Auto full strip");
    comboDetectionMode->Append("Full scan area");
    comboDetectionMode->Append("Stouffer T2115");
    detectionSizer->Add(labelFramesDetection, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    detectionSizer->Add(comboDetectionMode, 1);
    leftSizer->Add(detectionSizer, 0, wxEXPAND|wxALL, 5);

    // --- Output type row ---
    wxBoxSizer* outputTypeSizer = new wxBoxSizer(wxHORIZONTAL);
    labelOutputType = new wxStaticText(mainPanel, wxID_ANY, "Output type:");
    outputType = new wxComboBox(mainPanel, wxID_ANY);
    outputType->Append("Expanded negative density values");
    outputType->Append("Unexpanded negative density values");
    outputType->Append("Raw scan");
    outputTypeSizer->Add(labelOutputType, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    outputTypeSizer->Add(outputType, 1);
    leftSizer->Add(outputTypeSizer, 0, wxEXPAND|wxALL, 5);

    // --- Profile row ---
    wxBoxSizer* profileSizer = new wxBoxSizer(wxHORIZONTAL);
    labelProfile = new wxStaticText(mainPanel, wxID_ANY, "Profile:");
    comboProfile = new wxComboBox(mainPanel, wxID_ANY);
    profileSizer->Add(labelProfile, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    profileSizer->Add(comboProfile, 1);
    leftSizer->Add(profileSizer, 0, wxEXPAND|wxALL, 5);

    // --- Output DPI + Interpolation row ---
    wxBoxSizer* outputDPISizer = new wxBoxSizer(wxHORIZONTAL);
    labelOutputDPI = new wxStaticText(mainPanel, wxID_ANY, "Output DPI:");
    comboOutputDPI = new wxComboBox(mainPanel, wxID_ANY);
    labelInterpolation = new wxStaticText(mainPanel, wxID_ANY, "Interpolation:");
    comboInterpolation = new wxComboBox(mainPanel, wxID_ANY);
    comboInterpolation->Append("Nearest");
    comboInterpolation->Append("Bilinear");
    comboInterpolation->Append("Bicubic");
    comboInterpolation->Append("Area");
    comboInterpolation->Append("Lanczos");
    comboInterpolation->Append("Exact bilinear");
    comboInterpolation->Append("Exact nearest");
    outputDPISizer->Add(labelOutputDPI, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    outputDPISizer->Add(comboOutputDPI, 1, wxRIGHT, 5);
    outputDPISizer->Add(labelInterpolation, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    outputDPISizer->Add(comboInterpolation, 1);
    leftSizer->Add(outputDPISizer, 0, wxEXPAND|wxALL, 5);

    // --- Exposure compensation row ---
    wxBoxSizer* exposureSizer = new wxBoxSizer(wxHORIZONTAL);
    labelExposureComp = new wxStaticText(mainPanel, wxID_ANY, "Exposure comp:");
    brightness = new wxSpinCtrl(mainPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxSP_ARROW_KEYS, -3, 3, 0);
    exposureSizer->Add(labelExposureComp, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    exposureSizer->Add(brightness, 1);
    leftSizer->Add(exposureSizer, 0, wxEXPAND|wxALL, 5);

    // --- Output folder row ---
    wxBoxSizer* folderSizer = new wxBoxSizer(wxHORIZONTAL);
    labelOutputFolder = new wxStaticText(mainPanel, wxID_ANY, "Output Folder:");
    folderName = new wxTextCtrl(mainPanel, wxID_ANY, wxEmptyString, wxDefaultPosition, wxDefaultSize, wxTE_READONLY);
    toolSelectFolder = new wxButton(mainPanel, wxID_ANY, "...");
    folderSizer->Add(labelOutputFolder, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    folderSizer->Add(folderName, 1, wxRIGHT, 5);
    folderSizer->Add(toolSelectFolder, 0);
    leftSizer->Add(folderSizer, 0, wxEXPAND|wxALL, 5);

    // --- Roll name row ---
    wxBoxSizer* rollNameSizer = new wxBoxSizer(wxHORIZONTAL);
    labelRollName = new wxStaticText(mainPanel, wxID_ANY, "Roll name:");
    rollName = new wxTextCtrl(mainPanel, wxID_ANY);
    rollNameSizer->Add(labelRollName, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    rollNameSizer->Add(rollName, 1);
    leftSizer->Add(rollNameSizer, 0, wxEXPAND|wxALL, 5);

    // --- First frame n° row ---
    wxBoxSizer* firstFrameSizer = new wxBoxSizer(wxHORIZONTAL);
    labelFirstFrame = new wxStaticText(mainPanel, wxID_ANY, "First frame n°:");
    startingIndex = new wxComboBox(mainPanel, wxID_ANY);
    startingIndex->Append("XX");
    startingIndex->Append("X");
    startingIndex->Append("00");
    startingIndex->Append("0");
    startingIndex->Append("1");
    firstFrameSizer->Add(labelFirstFrame, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    firstFrameSizer->Add(startingIndex, 1);
    leftSizer->Add(firstFrameSizer, 0, wxEXPAND|wxALL, 5);

    // --- Multi-scan row ---
    wxBoxSizer* batchSizer = new wxBoxSizer(wxHORIZONTAL);
    labelBatch = new wxStaticText(mainPanel, wxID_ANY, "Multi-scan:");
    checkBatch = new wxCheckBox(mainPanel, wxID_ANY, "");
    comboBatch = new wxComboBox(mainPanel, wxID_ANY);
    comboBatch->Append("2");
    comboBatch->Append("4");
    comboBatch->Append("8");
    comboBatch->Append("16");
    batchSizer->Add(labelBatch, 0, wxALIGN_CENTER_VERTICAL|wxRIGHT, 5);
    batchSizer->Add(checkBatch, 0, wxRIGHT, 5);
    batchSizer->Add(comboBatch, 1);
    leftSizer->Add(batchSizer, 0, wxEXPAND|wxALL, 5);

    // --- Progress bar ---
    progressBar = new wxGauge(mainPanel, wxID_ANY, 100);
    leftSizer->Add(progressBar, 0, wxEXPAND|wxALL, 5);

    // --- Pulsanti principali ---
    wxBoxSizer* buttonSizer = new wxBoxSizer(wxHORIZONTAL);
    buttonOptions = new wxButton(mainPanel, wxID_ANY, "Options");
    buttonScan = new wxButton(mainPanel, wxID_ANY, "Scan");
    buttonCancel = new wxButton(mainPanel, wxID_ANY, "Cancel");
    buttonCancel->Disable();
    buttonSizer->Add(buttonOptions, 0, wxRIGHT, 5);
    buttonSizer->Add(buttonScan, 0, wxRIGHT, 5);
    buttonSizer->Add(buttonCancel, 0);
    leftSizer->Add(buttonSizer, 0, wxEXPAND|wxALL, 5);

    // --- Spazio ---
    leftSizer->AddStretchSpacer(1);

    // --- Area anteprima ---
    preview = new wxStaticBitmap(mainPanel, wxID_ANY, wxNullBitmap, wxDefaultPosition, wxSize(550, 330));
    rightSizer->Add(preview, 1, wxEXPAND|wxALL, 5);

    mainSizer->Add(leftSizer, 0, wxEXPAND);
    mainSizer->Add(rightSizer, 1, wxEXPAND);
    mainPanel->SetSizer(mainSizer);

    // Bind eventi
    buttonScan->Bind(wxEVT_BUTTON, &DensoScanFrame::OnScan, this);
    buttonCancel->Bind(wxEVT_BUTTON, &DensoScanFrame::OnCancel, this);
    buttonOptions->Bind(wxEVT_BUTTON, &DensoScanFrame::OnOptions, this);
    buttonRefresh->Bind(wxEVT_BUTTON, &DensoScanFrame::OnRefresh, this);
    toolSelectFolder->Bind(wxEVT_BUTTON, &DensoScanFrame::OnSelectFolder, this);
}

DensoScanFrame::~DensoScanFrame() {
    // Cleanup se necessario
}

// Esempio di event handler
void DensoScanFrame::OnScan(wxCommandEvent& event) {
    // Logica per avviare la scansione
}

void DensoScanFrame::OnCancel(wxCommandEvent& event) {
    // Logica per annullare la scansione
}

void DensoScanFrame::OnRefresh(wxCommandEvent&) {
    // TODO: logica per refresh device
}

void DensoScanFrame::OnSelectFolder(wxCommandEvent&) {
    // TODO: logica per selezione cartella
}

void DensoScanFrame::OnOptions(wxCommandEvent&) {
    // TODO: logica per opzioni
}

wxBEGIN_EVENT_TABLE(DensoScanFrame, wxFrame)
    // Esempio:
    // EVT_BUTTON(wxID_ANY, DensoScanFrame::OnScan)
wxEND_EVENT_TABLE()
