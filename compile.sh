g++-10 -std=c++20 -O3 -m64 -I/usr/include/SFML -c visual-attention.cpp -o visual-attention.o
g++-10 -o visual-attention visual-attention.o  -s -O3 -m64 -lpthread -lsfml-graphics -lsfml-window -lsfml-system

