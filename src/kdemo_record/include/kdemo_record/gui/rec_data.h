#ifndef REC_DATA_H
#define REC_DATA_H

#include <vector>
#include <string>
#include <armadillo>
#include <fstream>
#include <iomanip>

#include "utils.h"

class RecData
{
public:

    RecData();

    void addData(DataType::ID dat_id, const arma::vec &datum)
    { data[dat_id] = arma::join_horiz(data[dat_id], datum); }

    void addData(DataType::ID dat_id, const arma::mat &datum)
    { data[dat_id] = arma::join_horiz(data[dat_id], arma::vectorise(datum)); }

    bool isEmpty() const;
    bool isEmpty(DataType::ID dat_id) const
    { return data[dat_id].size()==0; }

    const arma::mat *getData(DataType::ID dat_id) const
    { return &data[dat_id]; }

    std::vector<arma::mat> data;

    bool saveData(const std::string &file_name) const;

    void clearData();

    std::string getErrMsg() const { return err_msg; }

private:
    std::string err_msg;

    template <typename T>
    void write_scalar(T scalar, std::ostream &out = std::cout, bool binary = false, int precision = 6) const
    {
      if (binary) out.write((const char *)(&scalar), sizeof(scalar));
      else out << std::setprecision(precision) << scalar << "\n";
    }

    void write_mat(const arma::mat &m, int n_rows, int n_cols, std::ostream &out, bool binary, int precision=6) const;
    void write_mat(const arma::mat &m, std::ostream &out, bool binary, int precision=6) const;

};


#endif // REC_DATA_H
