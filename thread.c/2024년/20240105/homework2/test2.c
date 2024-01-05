#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>

pthread_mutex_t mutex;

int sum;

void *func1(void *arg) 
{
    
    for (int i = 1; i <= 100; i++) 
    {
        pthread_mutex_lock(&mutex);
        sum += i;
        pthread_mutex_unlock(&mutex);
    }
    fprintf(stderr, "1부터 100까지의 합: %d\n", sum);
    pthread_exit(NULL);
}

void *func2(void *arg)
{
    for (int i = 101; i <= 200; i++) 
    {
        pthread_mutex_lock(&mutex);
        sum += i;
        pthread_mutex_unlock(&mutex);
    }
    fprintf(stderr, "1부터 200까지의 합: %d\n", sum);
    pthread_exit(NULL);
}

void *func3(void *arg)
{
    for (int i = 201; i <= 300; i++) 
    {
        pthread_mutex_lock(&mutex);
        sum += i; 
        pthread_mutex_unlock(&mutex);
    }
    fprintf(stderr, "1부터 300까지의 합: %d\n", sum);
    pthread_exit(NULL);
}

int main() 
{
    pthread_t tid1, tid2, tid3;

    sum= 0;


    if (pthread_create(&tid1, NULL, func1, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid2, NULL, func2, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_create(&tid3, NULL, func3, NULL) != 0) 
    {
        fprintf(stderr, "pthread create error\n");
        exit(1);
    }

    if (pthread_join(tid1, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid2, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }

    if (pthread_join(tid3, NULL) != 0) 
    {
        fprintf(stderr, "pthread join error\n");
        exit(1);
    }


    exit(0);
}
