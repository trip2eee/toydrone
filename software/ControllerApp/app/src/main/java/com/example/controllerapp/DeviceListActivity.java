package com.example.controllerapp;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.graphics.Color;
import android.os.Bundle;
import android.view.ViewGroup;
import android.view.Window;
import android.view.View;
import android.widget.AdapterView;
import android.widget.ArrayAdapter;
import android.widget.ListView;
import android.widget.TextView;

import java.util.ArrayList;

public class DeviceListActivity extends Activity {

    ArrayList<String> mArrayList;
    ArrayAdapter mAdapter;

    private BroadcastReceiver mReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {

            String strName = intent.getStringExtra("device_name");
            String strAddr = intent.getStringExtra("device_addr");
            System.out.println("Device received: " + strName + " - " + strAddr);

            // Send data?
            mArrayList.add(strName + "(" + strAddr + ")");
            mAdapter.notifyDataSetChanged();

        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);
        requestWindowFeature(Window.FEATURE_NO_TITLE);
        setContentView(R.layout.activity_device_list);

        mArrayList = new ArrayList<String>();

        mAdapter = new ArrayAdapter(this, android.R.layout.simple_list_item_1, mArrayList) {
            @Override
            public View getView(int position, View convertView, ViewGroup parent)
            {
                View view = super.getView(position, convertView, parent);
                TextView tv = (TextView) view.findViewById(android.R.id.text1);
                tv.setTextColor(Color.BLACK);
                return view;
            }

        };

        ListView listDevices = (ListView)findViewById(R.id.listDevices);
        listDevices.setAdapter(mAdapter);

        // List device item click event listener.
        listDevices.setOnItemClickListener(new AdapterView.OnItemClickListener() {
            @Override
            public void onItemClick(AdapterView<?> parent, View view, int position, long id) {

                Intent intent = new Intent("com.example.controllerapp.device_sel");
                intent.putExtra("id", id);

                final String text = (String) ((TextView)view).getText();// second method
                final String strName = text.substring(0, text.length() - 19);
                final String strAddr = text.substring(text.length() - 18, text.length()-1);

                intent.putExtra("text", text);
                intent.putExtra("name", strName);
                intent.putExtra("addr", strAddr);

                sendBroadcast(intent);

                // close activity.
                finish();
            }
        });

        // Intent receiver to receive Bluetooth device names.
        IntentFilter filter = new IntentFilter("com.example.controllerapp.device_name");
        registerReceiver(mReceiver, filter);

    }


    protected void onDestroy() {
        unregisterReceiver(mReceiver);

        super.onDestroy();
    }

    // Cancel button event handler.
    public void onCancel(View v)
    {
        // Close this activity.
        finish();
    }
}
