using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Hardware;
using CommandTypes;
using System.Diagnostics;

namespace Firmware
{

    public class FirmwareController
    {
        const string FIRMWARE_VERSION = "1.0";
        const float MAX_STEPPER_ACCEL = 4.0f;
        const float MAX_STEPPER_VELOCITY = 40.0f;
        const long TIMEOUT_THRESHOLD = 100000000L;
        const byte HEADER_SIZE = 4;
        const byte ACK_BYTE = 0xA5;
        const byte NAK_BYTE = 0xFF;

        bool fDone = false;
        bool fInitialized = false;
        PrinterControl printer;
        float plateZ = -1;
        long accelCounter = 0;
        float accelCurVelocity = 1.0f; // mm/s

        public FirmwareController(PrinterControl printer)
        {
            this.printer = printer;
        }

        String DoCommand(byte cmd, byte[] cmdParams, byte offset)
        {
            string retString = "";

            switch (cmd)
            {
                case FWCommands.CMD_VERSION:
                    retString = FIRMWARE_VERSION;
                    break;
                case FWCommands.CMD_RETRACT_PLATE:
                    RetractPlate();
                    retString = "SUCCESS";
                    break;

                case FWCommands.CMD_SET_Z:
                    SetZ(GetFloatParam(cmdParams, offset));
                    retString = "SUCCESS";
                    break;

                case FWCommands.CMD_REMOVE_MODEL:
                    printer.RemoveModelFromPrinter();
                    retString = "SUCCESS";
                    break;

                case FWCommands.CMD_MOVE_GALVOS:
                    printer.MoveGalvos(GetFloatParam(cmdParams, offset), GetFloatParam(cmdParams, offset + 4));
                    retString = "SUCCESS";
                    break;

                case FWCommands.CMD_SET_LASER:
                    if (cmdParams[offset] == 1) printer.SetLaser(true);
                    else printer.SetLaser(false);
                    retString = "SUCCESS";
                    break;

                default:
                    retString = "ERROR";
                    break;
            }

            return retString;
        }
        void Process()
        {
            byte[] value = new byte[HEADER_SIZE];
            byte[] cmdParams = new byte[64];
            int bytesRead = 0;
            string retString = "";
            int timeout;

            while (!fDone)
            {
                do    // Wait for command byte
                {
                    bytesRead = printer.ReadSerialFromHost(value, HEADER_SIZE);   // Read command, length, and checksum

                } while (!fDone && bytesRead == 0);

//                Console.WriteLine("Process: {0} {1}", value[0], value[1]);

                if (!fDone && bytesRead == HEADER_SIZE)   // Successfully read command byte, length byte, and checksum bytes
                {
                    // Write header back to host
                    printer.WriteSerialToHost(value, HEADER_SIZE);

                    byte[] ACKNAK = new byte[2];
                    ACKNAK[0] = 0;
                    do    // Wait for ACK/NACK byte
                    {
                        bytesRead = printer.ReadSerialFromHost(ACKNAK, 1);   // Read ACK/NAK

                    } while (!fDone && bytesRead == 0);

                    if (ACKNAK[0] == ACK_BYTE)
                    {
                        timeout = 0;
                        if (value[1] > 0)
                            do { timeout++; } while ((bytesRead = printer.ReadSerialFromHost(cmdParams, value[1])) == 0 && timeout < TIMEOUT_THRESHOLD);    // Read in the parameters

                        if (timeout == TIMEOUT_THRESHOLD)
                        {
                            retString = "TIMEOUT";
                        }
                        else
                        {
                            // Validate checksum
                            short checkSum = (byte)(value[0] + value[1]);
                            short targetCheckSum = (short)(((short)value[3] << 8) + value[2]);

                            for (int i = 0; i < value[1]; i++)
                                checkSum += cmdParams[i];

                            if (checkSum == targetCheckSum)  // Valid checksum, proceed
                            {

                                if (value[0] != FWCommands.CMD_BUFFER)
                                    retString = DoCommand(value[0], cmdParams, 0);
                                else  // Process command buffer
                                {
                                    while (cmdParams[0] != FWCommands.CMD_INVALID)
                                    {

                                        byte Ndx = 0;
                                        byte bufferLen = value[1];
                                        byte cmd = cmdParams[Ndx++];
                                        byte len = cmdParams[Ndx++];
                                        Ndx += len;

                                        retString = DoCommand(cmd, cmdParams, 2);

                                        // Delete used command
                                        for (byte i = 0; i < bufferLen - (len + 2); i++)
                                            cmdParams[i] = cmdParams[len + 2 + i];

                                        for (byte i = (byte)(bufferLen - (len + 2)); i < bufferLen; i++)
                                            cmdParams[i] = FWCommands.CMD_INVALID;
                                    }
                                }

                            }
                            else retString = "CHECKSUM";

                        } // If no TIMEOUT

                        printer.WriteSerialToHost(retString.Select(c => (byte)c).ToArray(), retString.Length);
                        value[0] = 0;
                        printer.WriteSerialToHost(value, 1);

                    }  // if ACK'd
                }
            }
        }

        public void Start()
        {
           fInitialized = true;

           Process(); // this is a blocking call
        }

        public void Stop()
        {
            fDone = true;
        }

        public void WaitForInit()
        {
            while (!fInitialized)
                Thread.Sleep(100);
        }

        void ResetAccelStepper()
        {
            accelCounter = 0;
            accelCurVelocity = 1.0f;
        }
        void WaitForNextStep()
        {
            long microWait;   

            microWait = (long)(1000000L / (accelCurVelocity * PrinterControl.STEPS_PER_MM));

            printer.WaitMicroseconds(microWait);

            accelCounter += microWait;

            if (accelCounter > 1000000L)  // Increase velocity every second
            {
                accelCurVelocity += MAX_STEPPER_ACCEL;
                if (accelCurVelocity > MAX_STEPPER_VELOCITY) accelCurVelocity = MAX_STEPPER_VELOCITY;

                accelCounter = 0;
            }
        }

        void RetractPlate()
        {
            ResetAccelStepper();


            while (!printer.LimitSwitchPressed())
            {
                if (!printer.StepStepper(PrinterControl.StepperDir.STEP_UP)) break;

                WaitForNextStep();

            }

            plateZ = printer.GetPrinterHeight();

        }

        float GetFloatParam(byte[] floatArr, int offset)
        {
            float retValue = System.BitConverter.ToSingle(floatArr, offset);

            return retValue;
        }

        // Set the Z height of the platform - Zero is at the bottom, against the resin tray
        void SetZ(float Z)
        {
            if (plateZ < 0)
            {
                RetractPlate();
            }

            float curZ = plateZ;
            PrinterControl.StepperDir dir = PrinterControl.StepperDir.STEP_UP;
            float diffZ = Z - curZ;

            ResetAccelStepper();

            if (diffZ < 0)
                dir = PrinterControl.StepperDir.STEP_DOWN;

            diffZ = Math.Abs(diffZ);
            int numSteps = (int) (diffZ * PrinterControl.STEPS_PER_MM);

            for (int i = 0; i < numSteps; i++)
            {
                printer.StepStepper(dir);

                WaitForNextStep();

                if (printer.LimitSwitchPressed()) break;
            }

            plateZ = Z;

        }
    }
}
