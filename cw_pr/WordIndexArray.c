#include "WordIndexArray.h"

WordIndexArray createDefaultWordIndexArray() {
    return (WordIndexArray){.inds = NULL, .size = 0};
}

void initDefaultWordIndexArray(WordIndexArray *array) {
    *array = createDefaultWordIndexArray();
}

void freeWordIndexArray(WordIndexArray *array) {
    free(array->inds);
    array->inds = NULL;
    array->size = 0;
}

void addNewWordIndex(WordIndexArray *array, WordIndex newWordIndex) {
    array->size++;
    array->inds = realloc(array->inds, array->size * sizeof(WordIndex));
    array->inds[array->size - 1] = newWordIndex;
}

void printWordIndexArray(WordIndexArray array) {
    printf("Words count: %lu\n", array.size);
    for (size_t i = 0; i < array.size; i++) {
        printf("%lu ", array.inds[i].index);
        printf("%lu\n", array.inds[i].lenght);
    }
}