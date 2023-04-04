using System;
using System.Collections.Generic;
using System.Data.SqlClient;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace stm_control
{
    internal class DbControl
    {
        private readonly string connectionStr = string.Format("Data Source={0},{1};Initial Catalog={2};User ID={3};Password={4}", "192.168.0.25", 1433, "DB_SMART_FARM", "sa", "8877");

        public void InsertTemperatureIntoDb(int temperature)
        {
            string insertSQL = string.Format(" INSERT INTO TB_TEMPERATURE ( DATETIME, TEMPERATURE ) VALUES ( '{0}', '{1}' ) ", DateTime.Now.ToString(), temperature);
            using (SqlConnection connection = new SqlConnection(connectionStr))
            {
                connection.Open();
                using (SqlCommand command = new SqlCommand(insertSQL, connection))
                {
                    command.ExecuteNonQuery();
                }
                connection.Close();
            }
        }

        public List<string> ReadTemperatureFromDb()
        {
            List<string> datas = new List<string>();
            string selectSQL = " SELECT ID, DATETIME, TEMPERATURE FROM TB_TEMPERATURE ";
            using (SqlConnection connection = new SqlConnection(connectionStr))
            {
                connection.Open();
                using (SqlCommand command = new SqlCommand(selectSQL, connection))
                {
                    SqlDataReader reader = command.ExecuteReader();
                    while (reader.Read())
                    {
                        datas.Add(reader[0] + "\t" + reader[1] + "\t" + reader[2] + "\u2103");
                    }
                }
                connection.Close();
            }
            return datas;
        }

        public int? DeleteTemperatureFromDb(int deleteId)
        {
            int? rowsAffected = null;
            string deleteSQL = string.Format(" DELETE FROM TB_TEMPERATURE WHERE ID='{0}' ", deleteId);
            using (SqlConnection connection = new SqlConnection(connectionStr))
            {
                connection.Open();
                using (SqlCommand command = new SqlCommand(deleteSQL, connection))
                {
                    rowsAffected = command.ExecuteNonQuery();
                }
                connection.Close();
            }
            return rowsAffected;
        }
    }
}
