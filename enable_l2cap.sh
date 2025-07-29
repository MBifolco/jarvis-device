#!/bin/bash

# Enable L2CAP COC in ESP32 configuration

echo "Enabling L2CAP Connection-Oriented Channels..."

# Update sdkconfig
sed -i 's/CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM=0/CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM=2/g' sdkconfig
sed -i 's/CONFIG_NIMBLE_L2CAP_COC_MAX_NUM=0/CONFIG_NIMBLE_L2CAP_COC_MAX_NUM=2/g' sdkconfig

# If lines don't exist, append them
if ! grep -q "CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM" sdkconfig; then
    echo "CONFIG_BT_NIMBLE_L2CAP_COC_MAX_NUM=2" >> sdkconfig
fi

if ! grep -q "CONFIG_NIMBLE_L2CAP_COC_MAX_NUM" sdkconfig; then
    echo "CONFIG_NIMBLE_L2CAP_COC_MAX_NUM=2" >> sdkconfig
fi

echo "L2CAP configuration updated. Run 'idf.py reconfigure' to apply changes."