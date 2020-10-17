/***
 * @file  MainActivity.java
 * @brief Main module.
 * @author trip2eee@gmail.com
 * @date 17, June, 2020
 */

package com.example.controllerapp;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import android.Manifest;
import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;

import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.os.Bundle;

import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;
import android.widget.ImageButton;

public class MainActivity extends AppCompatActivity {


    BLEComm mBLEComm;
    private static final int REQUEST_ENABLE_BT = 10;
    private static final int REQUEST_COARSE_LOCATION = 20;

    private static int mAltitudeZ = 100;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        TextView txtDebug = (TextView)findViewById(R.id.textDebug);
        txtDebug.setText("");

        // Set touch event listeners.
        ((ImageButton)findViewById(R.id.buttonUp)).setOnTouchListener(mTouchEvent);
        ((ImageButton)findViewById(R.id.buttonDown)).setOnTouchListener(mTouchEvent);

        ((ImageButton)findViewById(R.id.buttonForward)).setOnTouchListener(mTouchEvent);
        ((ImageButton)findViewById(R.id.buttonLeft)).setOnTouchListener(mTouchEvent);
        ((ImageButton)findViewById(R.id.buttonRight)).setOnTouchListener(mTouchEvent);
        ((ImageButton)findViewById(R.id.buttonBack)).setOnTouchListener(mTouchEvent);

        ((ImageButton)findViewById(R.id.buttonTurnLeft)).setOnTouchListener(mTouchEvent);
        ((ImageButton)findViewById(R.id.buttonTurnRight)).setOnTouchListener(mTouchEvent);

        //initializeBluetooth();
        mBLEComm = new BLEComm();
        
        // Before search for Bluetooth device, request ACCESS_COARSE_LOCATION permission.
        checkLocationPermission();
    }

    public void messageBox(String title, String message)
    {
        // No bluetooth adapter.
        AlertDialog.Builder alertDlgBuilder = new AlertDialog.Builder(this);

        alertDlgBuilder.setTitle(title);
        alertDlgBuilder.setMessage(message).setCancelable(false);
        alertDlgBuilder.setNegativeButton(getString(R.string.ok), new DialogInterface.OnClickListener() {
            @Override
            public void onClick(DialogInterface dialog, int which) {
                dialog.cancel();
            }
        });
        AlertDialog alertDlg = alertDlgBuilder.create();
        alertDlg.show();
    }

    protected void checkLocationPermission() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.ACCESS_COARSE_LOCATION)
                != PackageManager.PERMISSION_GRANTED) {

            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.ACCESS_COARSE_LOCATION},  REQUEST_COARSE_LOCATION);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode,
                                           String permissions[], int[] grantResults) {
        switch (requestCode) {
            case REQUEST_COARSE_LOCATION: {
                if (grantResults.length > 0
                        && grantResults[0] == PackageManager.PERMISSION_GRANTED) {

                    // If ACCESS_COARSE_LOCATION permission is granted.
                    initializeBluetooth();

                } else {
                    //TODO re-request
                }
                break;
            }
        }
    }

    public void initializeBluetooth()
    {
        final int result = mBLEComm.Initialize(this);
        if(result == -1)
        {
            Intent intent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(intent, REQUEST_ENABLE_BT);
        }
        else if(result == 0)
        {
            messageBox(getString(R.string.error), getString(R.string.plz_enable_bt));
        }
        else
        {
            mBLEComm.selectBluetoothDevice(this);

        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, @Nullable Intent data) {
        super.onActivityResult(requestCode, resultCode, data);

        switch(requestCode)
        {
            case REQUEST_ENABLE_BT:
                if(resultCode == RESULT_OK)
                {
                    mBLEComm.selectBluetoothDevice(this);
                }
                else
                {
                    messageBox(getString(R.string.error), getString(R.string.plz_enable_bt));
                }
                break;
        }
    }

    private ImageButton.OnTouchListener mTouchEvent = new ImageButton.OnTouchListener()
    {
        public boolean onTouch(View v, MotionEvent event) {
            int action = event.getAction();

            TextView txtDebug = (TextView)findViewById(R.id.textDebug);

            // if button is pressed.
            if(action == MotionEvent.ACTION_DOWN)
            {
                BLECommand cmd = new BLECommand();

                switch(v.getId()) {
                    case R.id.buttonUp:
                        txtDebug.setText("up pressed");

                        mAltitudeZ += 10;

                        if(mAltitudeZ > 255)
                        {
                            mAltitudeZ = 255;
                        }
                        cmd.cmd = BLECommand.ALTITUDE;
                        cmd.val = (short)mAltitudeZ;
                        mBLEComm.sendData(cmd);

                        break;
                    case R.id.buttonDown:
                        txtDebug.setText("down pressed");

                        mAltitudeZ -= 10;

                        if(mAltitudeZ <= 0)
                        {
                            mAltitudeZ = 0;
                        }

                        cmd.cmd = BLECommand.ALTITUDE;
                        cmd.val = (short)mAltitudeZ;
                        mBLEComm.sendData(cmd);

                        break;
                    case R.id.buttonForward:
                        txtDebug.setText("forward pressed");
                        //mBLEComm.sendData("1");

                        break;
                    case R.id.buttonLeft:
                        txtDebug.setText("left pressed");
                        break;
                    case R.id.buttonRight:
                        txtDebug.setText("right pressed");
                        break;
                    case R.id.buttonBack:
                        txtDebug.setText("back pressed");

                        //mBLEComm.sendData("2");
                        break;
                    case R.id.buttonTurnLeft:
                        txtDebug.setText("turn left pressed");
                        break;
                    case R.id.buttonTurnRight:
                        txtDebug.setText("turn right pressed");
                        break;
                }

            }
            else if (action == MotionEvent.ACTION_UP) {
                txtDebug.setText("button released");
            }

            return false;
        }
    };

    public void onButtonClick(View view)
    {
        TextView txtDebug = (TextView)findViewById(R.id.textDebug);

        switch(view.getId())
        {
            case R.id.buttonBlueTooth:
                txtDebug.setText("bluetooth");

                initializeBluetooth();

                break;
            case R.id.buttonRotorOn:
                txtDebug.setText("Rotor on");

                break;
        }

    }

    @Override
    protected void onDestroy()
    {
        mBLEComm.Finalize(this);

        super.onDestroy();
    }

}
