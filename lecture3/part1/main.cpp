#include <iostream>
using namespace std;

class Material
{
  Material();
  ~Material();
  void print();
};

Material::Material()
{
    cout << "Material Default Constructorl" << endl;
}

Material::~Material()
{
    cout << "Material Destructor" << endl;
}

Material::print()
{
    cout << "This is a Material object!" << ebdk;
}


int main()
{
    cout << "--- 构造函数 ---" << endl;
    Material m;

    cout << "\n--- print函数 ---" << endl;
    m.print();

    cout << "\n--- 自动进行析构函数 ---" << endl;
    return 0;
}