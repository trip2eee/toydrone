/***
 * @file  BluetoothComm.java
 * @brief Bluetooth communication.
 * @author trip2eee@gmail.com
 * @date 17, June, 2020
 * @reference https://yeolco.tistory.com/80
 */

package com.example.controllerapp;

import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.Context;
import android.content.DialogInterface;
import android.os.Handler;
import android.content.res.Resources;

import androidx.appcompat.app.AlertDialog;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.UUID;

public class BluetoothComm {

    private BluetoothAdapter bluetoothAdapter;              // Bluetooth adapter.
    private Set<BluetoothDevice> devices;                   // Bluetooth device set.
    private BluetoothDevice bluetoothDevice;
    private BluetoothSocket bluetoothSocket;
    private OutputStream outputStream = null;
    private InputStream inputStream = null;
    private Thread workerThread = null;
    private byte[] readBuffer;
    private int readBufferPosition;

    private boolean connected = false;

    public int Initialize()
    {
        int result;

        bluetoothAdapter = BluetoothAdapter.getDefaultAdapter();

        // If bluetooth adapter is available.
        if(bluetoothAdapter != null)
        {
            // If bluetooth is turned on.
            if(bluetoothAdapter.isEnabled())
            {
                result = 1;
            }
            else
            {
                result = -1;
            }
        }
        else
        {
            result = 0;
        }


        return result;
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

    public void selectBluetoothDevice(final Context context) {

        devices = bluetoothAdapter.getBondedDevices();

        final int pairedDeviceCount = devices.size();

        // If no paired Bluetooth device.
        if (pairedDeviceCount == 0) {
            messageBox(context, context.getString(R.string.error), context.getString(R.string.no_paired_dev));
        }
        // If paired devices exist.
        else {
            AlertDialog.Builder builder = new AlertDialog.Builder(context);
            builder.setTitle(context.getString(R.string.bt_dev_list));

            List<String> list = new ArrayList<>();

            for (BluetoothDevice bluetoothDevice : devices) {
                list.add(bluetoothDevice.getName());
            }

            // list to CharSequence
            final CharSequence[] charSequences = list.toArray(new CharSequence[list.size()]);
            list.toArray(new CharSequence[list.size()]);

            builder.setItems(charSequences, new DialogInterface.OnClickListener() {

                @Override
                public void onClick(DialogInterface dialog, int which) {
                    // connect to the corresponding device.
                    connected = connectDevices(charSequences[which].toString());

                    if(connected == false)
                    {
                        messageBox(context, context.getString(R.string.error), context.getString(R.string.connection_fail) + charSequences[which]);
                    }
                }
            });

            builder.setCancelable(true);
            builder.setNegativeButton(context.getString(R.string.cancel), new DialogInterface.OnClickListener() {
                @Override
                public void onClick(DialogInterface dialog, int which) {
                    dialog.cancel();
                }
            });
            AlertDialog alertDialog = builder.create();
            alertDialog.show();
        }
    }


    public boolean connectDevices(String deviceName)
    {
        boolean result = true;

        for(BluetoothDevice tempDevice: devices)
        {
            if(deviceName.equals(tempDevice.getName()))
            {
                bluetoothDevice = tempDevice;
                break;
            }
        }

        // UUID for Bluetooth communication.
        UUID uuid = java.util.UUID.fromString("00001101-0000-1000-8000-00805f9b34fb");

        // Create socket to communicate with the target device
        try {
            bluetoothSocket = bluetoothDevice.createRfcommSocketToServiceRecord(uuid);
            bluetoothSocket.connect();

            outputStream = bluetoothSocket.getOutputStream();
            inputStream = bluetoothSocket.getInputStream();

            receiveData();
        } catch(IOException e) {
            e.printStackTrace();

            result = false;
        }

        return result;
    }

    public void receiveData()
    {
        final Handler handler = new Handler();

        readBufferPosition = 0;
        readBuffer = new byte[1024];

        workerThread = new Thread(new Runnable() {
           @Override
           public void run()
           {
               while(Thread.currentThread().isInterrupted()){
                   try
                   {
                       // check if data is received.
                       int byteAvailable = inputStream.available();

                       // if data is received.
                       if(byteAvailable > 0)
                       {
                            byte[] bytes = new byte[byteAvailable];
                            inputStream.read(bytes);

                            for(int i = 0; i  <byteAvailable; i++)
                            {
                                byte tempByte = bytes[i];

                                if(tempByte == '\n')
                                {
                                    // copy readBuffer array to encodedBytes.
                                    byte[] encodedBytes = new byte[readBufferPosition];
                                    System.arraycopy(readBuffer, 0, encodedBytes, 0, encodedBytes.length);

                                    // convert encoded byte array into string.
                                    final String text = new String(encodedBytes, "US-ASCII");
                                    readBufferPosition = 0;

                                    System.out.println(text);
                                    /*
                                    handler.post(new Runnable() {

                                    });

                                    */
                                }
                                // If not new line character.
                                else
                                {
                                    readBuffer[readBufferPosition++] = tempByte;
                                }
                            }
                       }
                   } catch(IOException e) {
                       e.printStackTrace();
                   }

                   try {
                       // receive every 1 sec.
                       Thread.sleep(1000);

                   } catch(InterruptedException e)
                   {
                       e.printStackTrace();
                   }
               }
           }
        });

        workerThread.start();
    }

    public void sendData(String text)
    {
        try {
            outputStream.write(text.getBytes());
        } catch(Exception e) {
            e.printStackTrace();
        }
    }


}
