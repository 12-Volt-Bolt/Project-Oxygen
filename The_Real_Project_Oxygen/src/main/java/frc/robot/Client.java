/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.Socket;
import java.net.UnknownHostException;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Client {

    private Socket socket0 = null;
    private DataInputStream input = null;
    private DataOutputStream out = null;

    public Client(String address, int port) throws IOException {

        while (true) {
            try {
                socket0 = new Socket(address, port);

                // sends output to the socket
                out = new DataOutputStream(socket0.getOutputStream());
            } catch (UnknownHostException u) {
                System.out.println(u);
            } catch (IOException i) {
                System.out.println(i);
            }
        }
        // string to read message from input

        // keep reading until "Over" is input

        // close the connection
    }

}
