﻿using System.Text.Json;
using System.Text.Json.Nodes;

namespace JakaAPI.Types
{
    public class SendingResponse
    {
        public string CmdName { get; private set; }
        public string ErrorCode { get; private set; }
        public string ErrorMsg { get; private set; }

        public SendingResponse(string jsonString)
        {
            JsonObject jsonObject = JsonNode.Parse(jsonString)!.AsObject();
            CmdName = jsonObject["cmdName"]!.ToString();
            ErrorCode = jsonObject["errorCode"]!.ToString();
            ErrorMsg = jsonObject["errorMsg"]!.ToString();
        }
    }

    public class RobotData
    {
        public JointsPosition ArmJointsPosition { get; private set; }
        public Math.CartesianPosition ArmCartesianPosition { get; private set; }

        public RobotData(string rawJson)
        {            
            JsonObject jsonObject = JsonNode.Parse(rawJson)!.AsObject();

            double[] jointsArr = jsonObject["joint_actual_position"]!.AsArray().Deserialize<double[]>()!;
            ArmJointsPosition = new JointsPosition(jointsArr);

            double[] cartesianArr = jsonObject["actual_position"]!.AsArray().Deserialize<double[]>()!;
            ArmCartesianPosition = new Math.CartesianPosition(cartesianArr);
        }

        public override string ToString()
        {
            return ArmJointsPosition.ToString() + "\n" + ArmCartesianPosition.ToString();
        }
    }
}
