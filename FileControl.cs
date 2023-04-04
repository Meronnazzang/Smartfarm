using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace stm_control
{
    internal class FileControl
    {
        public const string DIRECTORY_SMART_FARM_FILES = "C:\\SmartFarmFiles";

        public DirectoryInfo CreateDirectory(string directoryName)
        {
            DirectoryInfo directoryToCreate = new DirectoryInfo(DIRECTORY_SMART_FARM_FILES + directoryName);
            if (directoryToCreate.Exists)
            {
                MessageBox.Show(directoryName + " " + Messages.DIRECTORY_EXISTS);
            }
            else
            {
                directoryToCreate.Create();
                MessageBox.Show(directoryName + " " + Messages.DIRECTORY_CREATED);
            }
            return directoryToCreate;
        }

        public FileInfo CreateDeleteLogFile(DirectoryInfo logDirectory, int deleteNumber)
        {
            string fileName = string.Format("\\{0}_delete_db.log", DateTime.Now.ToString("yyyy-MM-dd_HH-mm-ss"));
            FileInfo fileInfo = new FileInfo(logDirectory + fileName);
            using (StreamWriter sw = new StreamWriter(fileInfo.FullName))
            {
                sw.WriteLine("삭제된 데이터: {0}개", deleteNumber);
            }
            return fileInfo;
        }
    }
}
