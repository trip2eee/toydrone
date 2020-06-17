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

import android.app.AlertDialog;
import android.bluetooth.BluetoothAdapter;

import android.content.DialogInterface;
import android.content.Intent;
import android.os.Bundle;

import android.view.MotionEvent;
import android.view.View;
import android.widget.TextView;
import android.widget.ImageButton;

public class MainActivity extends AppCompatActivity {

    BluetoothComm btComm;
    private static final int REQUEST_ENABLE_BT = 10;

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

        initializeBluetooth();
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

    public void initializeBluetooth()
    {
        btComm = new BluetoothComm();
        final int result = btComm.Initialize();
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
            btComm.selectBluetoothDevice(this);

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
                    btComm.selectBluetoothDevice(this);
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
                switch(v.getId()) {
                    case R.id.buttonUp:
                        txtDebug.setText("up pressed");
                        break;
                    case R.id.buttonDown:
                        txtDebug.setText("down pressed");
                        break;
                    case R.id.buttonForward:
                        txtDebug.setText("forward pressed");

                        btComm.sendData("1");
                        break;
                    case R.id.buttonLeft:
                        txtDebug.setText("left pressed");
                        break;
                    case R.id.buttonRight:
                        txtDebug.setText("right pressed");
                        break;
                    case R.id.buttonBack:
                        txtDebug.setText("back pressed");

                        btComm.sendData("2");
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

}
