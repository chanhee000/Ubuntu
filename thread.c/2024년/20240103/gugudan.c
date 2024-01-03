#include <stdio.h>
#include <pthread.h>
#include <unistd.h>

// 쓰레드 동작시 실행될 함수
void *firstThreadRun(void *arg)
{
    
        printf("\n");
        sleep(1);
        for(int i = 1; i <= 9; i++)
        {

         printf("1 * %d = %d\n",i,1*i);
         sleep(1);
        }
    pthread_exit(NULL);
}

void *secondThreadRun(void *arg)
{

        sleep(10);
        printf("\n");
        for(int i = 1; i <= 9; i++)
        {

         printf("2 * %d = %d\n",i,2*i);
         sleep(1);
        }
    pthread_exit(NULL);
}

void *thirdThreadRun(void *arg)
{

        sleep(20);
        printf("\n");
        for(int i = 1; i <= 9; i++)
        {

         printf("3 * %d = %d\n",i,3*i);
         sleep(1);
        }
    pthread_exit(NULL);
}


int main()
{
    pthread_t firstThread, secondThread,thirdThread;
    int threadErr;

    // 쓰레드를 만들고 쓰레드 함수 실행
    if ((threadErr = pthread_create(&firstThread, NULL, firstThreadRun, NULL)))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d\n", threadErr);
    }

    if ((threadErr = pthread_create(&secondThread, NULL, secondThreadRun, NULL)))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d\n", threadErr);
    }
        if ((threadErr = pthread_create(&thirdThread, NULL, thirdThreadRun, NULL)))
    {
        // 에러시 에러 출력
        printf("Thread Err = %d\n", threadErr);
    }
   pthread_join(firstThread, NULL);
    pthread_join(secondThread, NULL);
    pthread_join(thirdThread, NULL);
}

