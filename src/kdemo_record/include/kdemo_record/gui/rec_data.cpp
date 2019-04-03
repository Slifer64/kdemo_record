#include "rec_data.h"

RecData::RecData()
{
    data.resize(DataType::getNumberOfDataTypes());
    err_msg = "";
}

void RecData::write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary, int precision) const
{
  if (binary)
  {
    double *buff = new double[n_rows*n_cols];

     int k=0;
     for (int i=0;i<n_rows;i++){
       for (int j=0;j<n_cols;j++) buff[k++] = m(i,j);
     }
     out.write((const char *)(buff), n_rows*n_cols*sizeof(double));

     delete []buff;
  }
  else
  {
    for (int i=0;i<n_rows;i++)
    {
      for (int j=0;j<n_cols;j++) out << std::setprecision(precision) << m(i,j) << " ";
      out << "\n";
    }
  }

}

void RecData::write_mat(const arma::mat &m, std::ostream &out, bool binary, int precision) const
{
    long n_rows = m.n_rows;
    long n_cols = m.n_cols;

    write_scalar(n_rows, out, binary);
    if (!binary) out << "\n";
    write_scalar(n_cols, out, binary);
    if (!binary) out << "\n";

    write_mat(m, n_rows, n_cols, out, binary, precision);
}

void RecData::clearData()
{
    for (unsigned i=0; i<data.size(); i++) data[i].clear();
}

bool RecData::isEmpty() const
{
    bool is_empty = true;

    for (int i=0; i<data.size(); i++)
    {
        if (!isEmpty(static_cast<DataType::ID>(i))) is_empty = false;
    }
    return is_empty;
}

bool RecData::saveData(const std::string &file_name) const
{
    if (isEmpty())
    {
        *(const_cast<std::string *>(&err_msg)) = "There are no recorded data!";
        return false;
    }

    std::ofstream out(file_name, std::ios::binary);
    if (!out)
    {
        *(const_cast<std::string *>(&err_msg)) = "Failed to open file \"" + file_name + "\".";
        return false;
    }

    for (unsigned i=0; i<data.size(); i++)
    {
        if (!isEmpty(static_cast<DataType::ID>(i)))
        {
            write_scalar(i, out, true);
            write_mat(data[i], out, true);
        }
    }
    out.close();

    return true;
}
