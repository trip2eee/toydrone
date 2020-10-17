/***
 * @file  BLEComm.java
 * @brief Bluetooth Low Energy (BLE) Communication.
 *        To use BLE the following permissions have to be declared in the manifest file.
 *          1. <uses-permission android:name="android.permission.BLUETOOTH"/>
 *          2. <uses-permission android:name="android.permission.BLUETOOTH_ADMIN"/>
 *          3. <uses-permission android:name="android.permission.ACCESS_FINE_LOCATION" /> (Required only if your app isn't using the Device Companion Manager.)
 *        To make this app available to BLE-capable devices only, include the following in the manifest file.
 *          <uses-feature android:name="android.hardware.bluetooth_le" android:required="true"/>
 * @author trip2eee@gmail.com
 * @date 17, June, 2020
 * @reference https://developer.android.com/guide/topics/connectivity/bluetooth-le#java
 */

package com.example.controllerapp;

import android.app.Service;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothClass;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothGatt;
import android.bluetooth.BluetoothGattCallback;
import android.bluetooth.BluetoothGattCharacteristic;
import android.bluetooth.BluetoothGattService;
import android.bluetooth.BluetoothProfile;

import android.bluetooth.le.ScanCallback;
import android.bluetooth.le.ScanResult;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;

import android.os.IBinder;

import android.util.Log;

import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Queue;
import java.util.Set;
import java.util.UUID;


// Characteristic process class
class CharaProc {

    private BluetoothGattCharacteristic mChara;
    private int mProcType;

    public CharaProc(BluetoothGattCharacteristic chara, int type) {
        mChara = chara;
        mProcType = type;
    }

    public int getProcType()
    {
        return mProcType;
    }

    public BluetoothGattCharacteristic getCharacteristic()
    {
        return mChara;
    }

}

class BLEProcQueue {
    public static final int TX = 0;
    public static final int RX = 1;

    private Queue<CharaProc> mProcQueue;

    public BLEProcQueue()
    {
        mProcQueue = new LinkedList<>();
    }

    public boolean add(BluetoothGattCharacteristic chara, int type)
    {
        CharaProc proc = new CharaProc(chara, type);
        return mProcQueue.add(proc);
    }

    public CharaProc poll()
    {
        CharaProc proc = mProcQueue.poll();
        return proc;
    }

}

class BLECommand {
    public static final int ROTOR_ON_OFF = 0;
    public static final int ALTITUDE = 1;

    public int cmd;
    public short val;
}

public class BLEComm extends Service {

    private BluetoothAdapter mBluetoothAdapter;              // Bluetooth adapter.

    private BluetoothGattCharacteristic mGattCharCommand;
    private BluetoothGattCharacteristic mGattCharRoll;
    private BluetoothGattCharacteristic mGattCharPitch;
    private BluetoothGattCharacteristic mGattCharYaw;
    private BluetoothGattCharacteristic mGattCharZ;

    private Set<BluetoothDevice> mDeviceSet;                   // Bluetooth device set.
    private BluetoothDevice mBluetoothDevice;

    private Intent mIntentDeviceList;

    private boolean connected = false;

    private boolean mReceiverRegistered = false;

    private final static String TAG = "BleComm";

    private BluetoothGatt mBluetoothGatt = null;        // General Attribute Profile (GATT).

    private BLEProcQueue mProcQueue = null;

    public final static UUID UUID_DRONE_SERVICE = UUID.fromString("0000180F-0000-1000-8000-00805F9B34FB");
    public final static UUID UUID_DRONE_COMMAND_CHARACTERISTIC = UUID.fromString("00002A10-0000-1000-8000-00805F9B34FB");
    public final static UUID UUID_DRONE_ROLL_CHARACTERISTIC = UUID.fromString("00002A20-0000-1000-8000-00805F9B34FB");
    public final static UUID UUID_DRONE_PITCH_CHARACTERISTIC = UUID.fromString("00002A30-0000-1000-8000-00805F9B34FB");
    public final static UUID UUID_DRONE_YAW_CHARACTERISTIC = UUID.fromString("00002A40-0000-1000-8000-00805F9B34FB");
    public final static UUID UUID_DRONE_Z_CHARACTERISTIC = UUID.fromString("00002A50-0000-1000-8000-00805F9B34FB");

    public int roll = -90;

    public int Initialize(final Context context)
    {
        int result;

        // Get the Bluetooth Adapter.
        //final BluetoothManager bluetoothManager = (BluetoothManager)getSystemService(context.BLUETOOTH_SERVICE);
        //mBluetoothAdapter = bluetoothManager.getAdapter();


        mBluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If bluetooth adapter is available.
        if(mBluetoothAdapter == null || mBluetoothAdapter.isEnabled() == false) {
            result = -1;
        }
        else
        {
            if(mReceiverRegistered == false) {
                IntentFilter searchFilter = new IntentFilter();
                searchFilter.addAction(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
                searchFilter.addAction(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
                searchFilter.addAction(BluetoothDevice.ACTION_FOUND);
                searchFilter.addAction("com.example.controllerapp.device_sel");
                context.registerReceiver(mBluetoothSearchReceiver, searchFilter);

                mReceiverRegistered = true;

                mProcQueue = new BLEProcQueue();
            }

            result = 1;
        }

        return result;
    }

    public int Finalize(final Context context)
    {
        if(mReceiverRegistered == true) {
            try {
                context.unregisterReceiver(mBluetoothSearchReceiver);

            }catch(IllegalArgumentException e)
            {
                e.printStackTrace();
            }

        }

        if(mBluetoothGatt != null)  {
            mBluetoothGatt.close();
            mBluetoothGatt = null;
        }
        return 0;
    }

    public void messageBox(Context context, String title, String message)
    {
        // No bluetooth adapter.
        android.app.AlertDialog.Builder alertDlgBuilder = new android.app.AlertDialog.Builder(context);

        alertDlgBuilder.setTitle(title);
        alertDlgBuilder.setMessage(message).setCancelable(false);
        alertDlgBuilder.setNegativeButton(context.getString(R.string.ok), new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.cancel();
            }
        });
        android.app.AlertDialog alertDlg = alertDlgBuilder.create();
        alertDlg.show();
    }

    private final BroadcastReceiver mBluetoothSearchReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();

            if(BluetoothAdapter.ACTION_DISCOVERY_STARTED.equals(action)) {

            }
            else if(BluetoothDevice.ACTION_FOUND.equals(action)) {
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                BluetoothClass.Device deviceClass = new BluetoothClass.Device();
                String name;
                name = device.getName();
                if(name != null) {

                    if(mDeviceSet.contains(device) == false) {
                        mDeviceSet.add(device);

                        final String address = device.getAddress();
                        System.out.println("Device found: " + name + " - " + address);

                        Intent iSend = new Intent("com.example.controllerapp.device_name");
                        iSend.putExtra("device_name", name.toString());
                        iSend.putExtra("device_addr", address.toString());
                        context.sendBroadcast(iSend);
                    }
                }

            }
            else if(BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {
                System.out.println(mDeviceSet.toString());

            }
            else if(action == "com.example.controllerapp.device_sel")  {
                String strText = intent.getStringExtra("text");
                long id = intent.getLongExtra("id", -1);

                final String strName = intent.getStringExtra("name");
                final String strAddr = intent.getStringExtra("addr");

                System.out.println("Device selected! name: " + strName + " addr: " + strAddr);
                mBluetoothAdapter.cancelDiscovery();

                connected = connectDevices(context, strName, strAddr);

                if(connected == false)
                {
                    messageBox(context, context.getString(R.string.error), context.getString(R.string.connection_fail) + strName);
                }

            }
            else {
                System.out.println("Receiver: " + action.toString());
            }
        }
    };

    private ScanCallback mScanCallback = new ScanCallback() {
        @Override
        public void onScanResult(int callbackType, ScanResult result) {
            super.onScanResult(callbackType, result);

            BluetoothDevice device = result.getDevice();
            System.out.println("BLE found : " + device.getName());
        }

        @Override
        public void onBatchScanResults(List<ScanResult> results) {
            super.onBatchScanResults(results);

            System.out.println("Batch scan results");
        }

        @Override
        public void onScanFailed(int errorCode) {
            super.onScanFailed(errorCode);

            if(errorCode == SCAN_FAILED_ALREADY_STARTED) {
                System.out.println("SCAN_FAILED_ALREADY_STARTED");
            }
            else if(errorCode == SCAN_FAILED_APPLICATION_REGISTRATION_FAILED) {
                System.out.println("SCAN_FAILED_APPLICATION_REGISTRATION_FAILED");
            }
            else if(errorCode == SCAN_FAILED_FEATURE_UNSUPPORTED) {
                System.out.println("SCAN_FAILED_FEATURE_UNSUPPORTED");
            }
            else if(errorCode == SCAN_FAILED_INTERNAL_ERROR) {
                System.out.println("SCAN_FAILED_INTERNAL_ERROR");
            }
        }
    };

    // Various callback methods defined by the BLE API.
    private final BluetoothGattCallback gattCallback = new BluetoothGattCallback() {
        @Override
        public void onConnectionStateChange(BluetoothGatt gatt, int status, int newState) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                Log.i(TAG, "Connected to GATT server.");
                Log.i(TAG, "Attempting to start service discovery:" +  mBluetoothGatt.discoverServices());

            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                Log.i(TAG, "Disconnected from GATT server.");
            }
        }

        @Override
        // New services discovered
        public void onServicesDiscovered(BluetoothGatt gatt, int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {
                Log.i(TAG, "BluetoothGatt.GATT_SUCCESS");

                // Query all the supported services and characteristics on the user interface.
                List<BluetoothGattService> listService = gatt.getServices();

                // show services.
                Log.i(TAG, "Sercvice list");

                for(BluetoothGattService service: listService ) {

                    // UUIDs have to be compared by equals() method.
                    if(service.getUuid().equals(UUID_DRONE_SERVICE)) {
                        String uuid = service.getUuid().toString();
                        Log.i(TAG, "UUID: " + uuid);

                        List<BluetoothGattCharacteristic> listGattCharas = service.getCharacteristics();

                        for (BluetoothGattCharacteristic gattChara : listGattCharas) {
                            if(gattChara.getUuid().equals(UUID_DRONE_COMMAND_CHARACTERISTIC)) {
                                mGattCharCommand = gattChara;
                                boolean enabled = true;
                                mBluetoothGatt.setCharacteristicNotification(mGattCharCommand, enabled);
                            }
                            else if(gattChara.getUuid().equals(UUID_DRONE_ROLL_CHARACTERISTIC)) {
                                mGattCharRoll = gattChara;
                                boolean enabled = true;
                                mBluetoothGatt.setCharacteristicNotification(mGattCharRoll, enabled);
                            }
                            else if(gattChara.getUuid().equals(UUID_DRONE_PITCH_CHARACTERISTIC)) {
                                mGattCharPitch = gattChara;
                                boolean enabled = true;
                                mBluetoothGatt.setCharacteristicNotification(gattChara, enabled);
                            }
                            else if(gattChara.getUuid().equals(UUID_DRONE_YAW_CHARACTERISTIC)) {
                                mGattCharYaw = gattChara;
                                boolean enabled = true;
                                mBluetoothGatt.setCharacteristicNotification(gattChara, enabled);
                            }
                            else if(gattChara.getUuid().equals(UUID_DRONE_Z_CHARACTERISTIC)) {
                                mGattCharZ = gattChara;
                                boolean enabled = true;
                                mBluetoothGatt.setCharacteristicNotification(gattChara, enabled);
                            }

                        }
                    }

                }

            } else {
                Log.w(TAG, "onServicesDiscovered received: " + status);
            }
        }

        @Override
        // Characteristic notification
        public void onCharacteristicChanged(BluetoothGatt gatt,
                                            BluetoothGattCharacteristic characteristic) {
            Log.w(TAG, "Bluetooth Gatt Characteristic changed");
        }

        @Override
        public void onCharacteristicWrite (BluetoothGatt gatt,
                                           BluetoothGattCharacteristic characteristic,
                                           int status)
        {
            if(status == BluetoothGatt.GATT_SUCCESS) {
                processCharacteristicQueue();
            }
        }

        @Override
        // Result of a characteristic read operation
        public void onCharacteristicRead(BluetoothGatt gatt,
                                         BluetoothGattCharacteristic characteristic,
                                         int status) {
            if (status == BluetoothGatt.GATT_SUCCESS) {

                byte[] val = characteristic.getValue();
                String strVal = new String(val);
                Log.i(TAG, "read: " + strVal);

                processCharacteristicQueue();
            }
        }
    };

    public void selectBluetoothDevice(final Context context)
    {
        mDeviceSet = new HashSet<BluetoothDevice>();

        mIntentDeviceList = new Intent(context, DeviceListActivity.class);

        ((AppCompatActivity)context).startActivity(mIntentDeviceList);
        try {
            Thread.sleep(500);
        } catch(InterruptedException e)
        {

        }

        Log.i(TAG, "Searching for unpaired devices...");
        // search for unpaired devices.
        mBluetoothAdapter.startDiscovery();

        //BluetoothLeScanner scanner = mBluetoothAdapter.getBluetoothLeScanner();
        //scanner.startScan(mScanCallback);

    }

    public boolean connectDevices(Context context, String strName, String strAddr)
    {
        boolean result = true;

        for(BluetoothDevice tempDevice: mDeviceSet)
        {
            // TODO: To check address.
            if(strName.equals(tempDevice.getName()) &&
               strAddr.equals(tempDevice.getAddress()))
            {
                mBluetoothDevice = tempDevice;
                break;
            }
        }

        // Create socket to communicate with the target device
        mBluetoothGatt = mBluetoothDevice.connectGatt(context, false, gattCallback);

        return result;
    }

    public void processCharacteristicQueue()
    {
        CharaProc proc = mProcQueue.poll();
        if(proc != null) {

            if(proc.getProcType() == BLEProcQueue.TX) {
                mBluetoothGatt.writeCharacteristic(proc.getCharacteristic());
            }
            else if(proc.getProcType() == BLEProcQueue.RX) {
                mBluetoothGatt.readCharacteristic(proc.getCharacteristic());
            }

        }
    }

    public void sendData(BLECommand cmd)
    {
        if(cmd.cmd == BLECommand.ALTITUDE)
        {
            mGattCharZ.setValue(cmd.val, BluetoothGattCharacteristic.FORMAT_SINT16, 0);
            mProcQueue.add(mGattCharZ, BLEProcQueue.TX);
        }
        /*
        roll += 1;

        if(mGattCharRoll.setValue(roll, BluetoothGattCharacteristic.FORMAT_SINT16, 0)) {

        }
        else {
            Log.i(TAG, "failed to set");
        }

        byte[] bytes = new byte[1];
        bytes[0] = 'r';
        mGattCharCommand.setValue(bytes);

        //mBluetoothGatt.writeCharacteristic(mGattCharRoll);
        //TxChara data = new TxChara(mGattCharRoll);

        mProcQueue.add(mGattCharRoll, BLEProcQueue.TX);
        mProcQueue.add(mGattCharCommand, BLEProcQueue.TX);
        mProcQueue.add(mGattCharCommand, BLEProcQueue.RX);

        mProcQueue.add(mGattCharZ, BLEProcQueue.TX);
        */
        processCharacteristicQueue();
    }


    @Nullable
    @Override
    public IBinder onBind(Intent intent) {
        return null;
    }
}
