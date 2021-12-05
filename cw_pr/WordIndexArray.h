#pragma once
#include <stdio.h>
#include <stdlib.h>

typedef struct WordIndex {
    size_t index;
    size_t lenght;
    char delim;
} WordIndex;

typedef struct WordIndexArray {
    WordIndex *inds;
    size_t size;
} WordIndexArray;

WordIndexArray createDefaultWordIndexArray();
void initDefaultWordIndexArray(WordIndexArray *array);
void freeWordIndexArray(WordIndexArray *array);

void addNewWordIndex(WordIndexArray *array, WordIndex newWordIndex);

void printWordIndexArray(WordIndexArray array);
