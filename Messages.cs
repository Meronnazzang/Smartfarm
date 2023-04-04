using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace stm_control
{
    internal class Messages
    {
        public const string PORT_NOT_OPEN = "포트가 열려 있지 않습니다.";
        public const string DIRECTION_NOT_CHOSEN = "회전 방향을 선택해 주십시오.";
        public const string PREVENT_RAPID_TURN = "급회전 방지를 위해, 정지 후 방향을 변경해야 합니다.";
        public const string FORMAT_EXCEPTION = "숫자가 아닌 값을 입력했습니다.";
        public const string DELETE_FROM_DB_OK = "데이터 삭제가 완료되었습니다.";
        public const string LOG_FILE_CREATED = "로그파일이 생성되었습니다.";
        public const string DIRECTORY_CREATED = "디렉토리가 생성되었습니다.";
        public const string DIRECTORY_EXISTS = "디렉토리가 이미 생성되어 있습니다.";
    }
}
