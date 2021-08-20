#include <QRFilter/QRFilter_dialog.hpp>

namespace iort_filters
{
    QRFilterDialog::QRFilterDialog(insitu::Filter *parent_) : FilterDialog(parent_)
    {
        okButton = new QPushButton(tr("OK"));
        okButton->setDefault(true);
        cancelButton = new QPushButton(tr("Cancel"));
        desc = new QLabel(tr("Data Member List"));
        list = new QListWidget();
        list->setSpacing(5);
        list->setDragEnabled(true);
        list->viewport()->setAcceptDrops(true);
        list->setDropIndicatorShown(true);
        list->setDragDropMode(QAbstractItemView::InternalMove);
        cb = new QCheckBox("Generate bar view (0-4095) for integer members");
        estimate = new QCheckBox("Estimate Intermediate Values");

        simLatencyLabel = new QLabel(tr("Simulated Latency (ms):"));
        simLatencyBox = new QSpinBox();
        simLatencyBox->setRange(0, 10000);
        simLatencyBox->setValue(0);
        thresholdLatencyLabel = new QLabel(tr("Threshold for Estimation (ms):"));
        thresholdLatencyBox = new QSpinBox();
        thresholdLatencyBox->setRange(0, 10000);
        thresholdLatencyBox->setValue(250);
        sampleSizeLabel = new QLabel(tr("Estimation Sample Size:"));
        sampleSizeBox = new QSpinBox();
        sampleSizeBox->setRange(0, 20);
        sampleSizeBox->setValue(10);
        weightParamLabel = new QLabel(tr("Weight Parameter:"));
        weightParamBox = new QDoubleSpinBox();
        weightParamBox->setRange(1.0, 10.0);
        weightParamBox->setSingleStep(0.1);
        weightParamBox->setValue(2);

        layout = new QGridLayout();
        layout->addWidget(desc, 0, 0, 1, 2);
        layout->addWidget(list, 1, 0, 1, 2);
        layout->addWidget(cb, 2, 0, 1, 2);
        layout->addWidget(simLatencyLabel, 3, 0);
        layout->addWidget(simLatencyBox, 3, 1);
        layout->addWidget(estimate, 4, 0, 1, 2);
        layout->addWidget(thresholdLatencyLabel, 5, 0);
        layout->addWidget(thresholdLatencyBox, 5, 1);
        layout->addWidget(sampleSizeLabel, 6, 0);
        layout->addWidget(sampleSizeBox, 6, 1);
        layout->addWidget(weightParamLabel, 7, 0);
        layout->addWidget(weightParamBox, 7, 1);
        layout->addWidget(cancelButton, 8, 0);
        layout->addWidget(okButton, 8, 1);
        

        setLayout(layout);

        QObject::connect(okButton, SIGNAL(clicked()), SLOT(onOK()));
        QObject::connect(cancelButton, SIGNAL(clicked()), SLOT(reject()));
    }

    void QRFilterDialog::onOK(void)
    {
        Json::Value &settings = parent->getSettingsValue();
        queries.clear();
        int bars = 0;
        for (int i = 0; i < list->count(); i++)
        {
            QListWidgetItem *item = list->item(i);
            if (item->checkState())
            {
                queries.push_back(item->text().toStdString());
                if (settings["data"].get(item->text().toStdString(), 0).isInt())
                {
                    bars++;
                }
            }
        }
        settings["bars"] = bars;
        settings["generate_bars"] = (bool)cb->checkState();
        settings["estimate"] = (bool)estimate->checkState();
        settings["sim_lat"] = simLatencyBox->value();
        settings["threshold"] = thresholdLatencyBox->value();
        settings["sample_size"] = sampleSizeBox->value();
        settings["weight_param"] = weightParamBox->value();
        accept();
    }

    void QRFilterDialog::updateList(void)
    {
        Json::Value &settings = parent->getSettingsValue();
        std::vector<std::string> members = settings["data"].getMemberNames();
        int bars = 0;
        for (int i = 0; i < members.size(); i++)
        {
            queries.push_back(members[i]);
            if (settings["data"].get(members[i], 0).isInt())
            {
                bars++; // count number of bars that need to be generated for scaling purposes
            }
            QListWidgetItem *item = new QListWidgetItem(members[i].c_str(), list);
            item->setFlags(item->flags() | Qt::ItemIsUserCheckable); // set checkable flag
            item->setCheckState(Qt::Checked);                        // AND initialize check state
            list->addItem(item); // Add widget to list
        }
        settings["bars"] = bars;
    }

} // end namespace iort_filters
