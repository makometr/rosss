#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "WordIndexArray.h"

char WORD_DELIMS[] = {',', ' ', '\n'};
char SENTENCE_DELIMS[] = {'.'};

// - Удалить каждое слово в предложении, состоящее из всех гласных букв
char *deleteAllWordsWithAllVowels(char const *const sentence, WordIndexArray array) {
    size_t new_size = 0;
    for (size_t i = 0; i < array.size; i++)
        new_size += array.inds[i].lenght + 1;
    new_size++;

    char *new_sentence = calloc(new_size, sizeof(char));
    char *last_ptr = new_sentence;
    for (size_t word_i = 0; word_i < array.size; word_i++) {
        memcpy(last_ptr, &sentence[array.inds[word_i].index], array.inds[word_i].lenght);
        last_ptr += array.inds[word_i].lenght;
        *last_ptr = array.inds[word_i].delim;
        last_ptr++;
    }

    return new_sentence;
}

// Функция, которая возвращаает слова (все в предложении)
WordIndexArray findAllWords(const char *const sentence) {
    // WordIndexArray indexArray = {.inds = NULL, .size = 0};
    WordIndexArray indexArray = createDefaultWordIndexArray();

    size_t begin_index = 0;
    for (size_t i = 0; i < strlen(sentence); i++) {
        if (strchr(WORD_DELIMS, sentence[i]) != NULL) {
            size_t word_size = i - begin_index;
            if (word_size == 0) {
                // Наткнулись на подряд идущие разделители
                begin_index++;
                continue;
            }
            addNewWordIndex(&indexArray, (WordIndex){.index = begin_index, .lenght = word_size, .delim = sentence[begin_index + word_size]});
            begin_index = i + 1;
        }
    }

    if (begin_index != strlen(sentence)) {
        size_t word_size = strlen(sentence) - begin_index;
        addNewWordIndex(&indexArray, (WordIndex){.index = begin_index, .lenght = word_size, .delim = '\0'});
    }
    return indexArray;
}

// Функция, которая говорит, какие из слов являются гласными
WordIndexArray filterVowelWords(const char *const s, WordIndexArray words) {
    // WordIndexArray filtered = {.inds = NULL, .size = 0};
    WordIndexArray filtered = createDefaultWordIndexArray();
    for (size_t word_index = 0; word_index < words.size; word_index++) {
        for (size_t char_index = words.inds[word_index].index; char_index < words.inds[word_index].index + words.inds[word_index].lenght; char_index++) {
            char cur_char = s[char_index];
            if (strchr("euioay", cur_char) == NULL) {
                addNewWordIndex(&filtered, words.inds[word_index]);
                break;
            }
        }
    }
    return filtered;
}

int main() {
    char *line = NULL;
    size_t size;
    getline(&line, &size, stdin);

    WordIndexArray index_array = findAllWords(line);
    printWordIndexArray(index_array);

    printf("Work!\n");
    WordIndexArray index_array_without_vowels = filterVowelWords(line, index_array);
    printWordIndexArray(index_array_without_vowels);
    char *newSentence = deleteAllWordsWithAllVowels(line, index_array_without_vowels);

    printf("Sentence after work:\n");
    printf("%s\n", newSentence);
    free(newSentence);
    // free(index_array.inds);
    // free(index_array_without_vowels.inds);
    freeWordIndexArray(&index_array);
    freeWordIndexArray(&index_array_without_vowels);
    free(line);
}