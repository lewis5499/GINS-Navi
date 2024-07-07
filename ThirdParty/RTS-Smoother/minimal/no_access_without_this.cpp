

class Child{
    public:
    double mu;
};


template< class _Child>
class Parent: public _Child{
    public:
    Parent():mu(0){

    }
};

class ParentWorks: public Child{
    public:
    ParentWorks():mu(0){

    }
};

int main(int argc, char *argv[]) {
    ParentWorks works;
    Parent<Child> works_not;
}