using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace CommandTypes
{
    public class FWCommands
    {
        public const byte CMD_INVALID = 0;
        public const byte CMD_VERSION = 10;
        public const byte CMD_RETRACT_PLATE = 11;
        public const byte CMD_SET_Z = 12;
        public const byte CMD_REMOVE_MODEL = 13;
        public const byte CMD_MOVE_GALVOS = 14;
        public const byte CMD_SET_LASER = 15;
        public const byte CMD_BUFFER = 16;

    }
}
