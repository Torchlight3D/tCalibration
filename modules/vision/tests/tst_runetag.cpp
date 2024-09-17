#include <gtest/gtest.h>

TEST(RuneTag, Coding)
{
#if 0
int main()
{
    timeval tv;
    gettimeofday(&tv,0);
    srand((unsigned)tv.tv_usec);

    init();

    std::vector<long> code;
    ///*
    while(std::cin)
    {
        unsigned char c;
        if (!(std::cin >> c)) break;

        switch (c)
        {
        case '-' :
            code.push_back(-1L);
            break;
        case '0' :
        case '1' :
        case '2' :
        case '3' :
        case '4' :
        case '5' :
        case '6' :
            code.push_back(static_cast<long>(c-'0'));
        }
    }
    //*/
    /*
    long rand_idx=rand()%num_words;
    long idx=generate(code,rand_idx);
    */

    long index,rotation;
    decode(code);

    std::cout << "\ncorrected code:\n[";
    for (long i=0; i!=code_length; ++i)
        std::cout << " " << code[i];
    std::cout << " ]\n\n";


    align(code, index, rotation);

    std::cout << "Aligned code\nIndex: " << index << //", idx: " << idx << ", rand_idx: " << rand_idx <<
                 ", rotation: " << rotation << std::endl;

    std::cout << "[";
    for (long i=0; i!=code_length; ++i)
        std::cout << " " << code[i];
    std::cout << " ]\n\n";

    return 0;
}

#endif
}
