#ifndef iort_filters_IoRT_Plot_DIALOG_HPP
    #define iort_filters_IoRT_Plot_DIALOG_HPP

    #include <insitu/filter.hpp>

    namespace iort_filters {

    class IoRT_PlotDialog : public insitu::FilterDialog
    {
    Q_OBJECT
    private:

        QPushButton * okButton;
        QPushButton * cancelButton;

        QGridLayout * layout;

    public Q_SLOTS:

        void onOK(void);

    public:
        IoRT_PlotDialog(insitu::Filter * parent_);

    };

    } // end namespace iort_filters

    #endif // end iort_filters_IoRT_Plot_DIALOG_HPP
    