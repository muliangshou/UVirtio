#include <stdarg.h>

/**
 * Compute dot product
 */
float dot(float *x, ...)
{
    va_list w;
    va_start(w, 4);
    float dot = 0.0;

    for (int i = 0; i < 4; i++) {
        const float wi = va_arg(w, double);
        dot += x[i] * wi;
    }

    return dot;
}

/**
 * Predict class for features vector
 */
int predict(float *x)
{
    float votes[3] = {0.0f};
    votes[0] = dot(x, -0.423405592418, 0.967388282125, -2.517050233286, -1.079182996654) + 9.84868307535428;
    votes[1] = dot(x, 0.534517184386, -0.321908835083, -0.206465997471, -0.944448257908) + 2.238120068472271;
    votes[2] = dot(x, -0.111111591968, -0.645479447042, 2.723516230758, 2.023631254562) + -12.086803143826813;
    // return argmax of votes
    int classIdx = 0;
    float maxVotes = votes[0];

    for (int i = 1; i < 3; i++) {
        if (votes[i] > maxVotes) {
            classIdx = i;
            maxVotes = votes[i];
        }
    }

    return classIdx;
}