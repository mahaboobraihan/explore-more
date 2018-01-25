//======================================================================
//
// Test program to test the infrared sensors (and motors) of the
// 4tronix initio robot car. One can run this program within an
// ssh session.
//
// author: Raimund Kirner, University of Hertfordshire
//         initial version: Dec.2016
//
// license: GNU LESSER GENERAL PUBLIC LICENSE
//          Version 2.1, February 1999
//          (for details see LICENSE file)
//
// Compilation: 
// gcc -o camcar -Wall -Werror -lcurses -lwiringPi -lpthread -linitio camcar.c
//
//======================================================================

#include <stdlib.h>
#include <initio.h>
#include <curses.h>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include "detect_blob.h"

//======================================================================
// Coursework ESD, general instructions
// This file (camcar.c) is the major file to be modified in order to 
// complete the coursework.  There are a few locations marked with "TODO",
// which indicate places where the code might need changes or completion.
// This directory also contains two other source files:
// quickblob.c ... this is a library for searching blobs in images
// detect_blob.c ... this is a wrapper for quickblob.c, providing a 
//                   direct interface to the RaspberryPI camera.
// Normally, quickblob.c and detect_blob.c don't need changes. However,
// studying detect_blob.c a bit is still advisable.
//
// The implementation of the nested state machine in camcar() follows
// the implementation proposal given in the Lecture slides. You may
// want to change the FSM implementation to add extra or refined
// behaviour.
//======================================================================



//======================================================================
// camcar():
// Skeleton code for the ESD coursework.
// The implementation uses hierarchical finite state machines (FSM), in order
// to reduce the size of the state transition graph.
// To be done: Fill the actions of the individual states of the FSMs
// with meaningful behaviour.
//======================================================================
void camcar(int argc, char *argv[]) 
{
    int ch = 0;

    // main control loop:  
    while (ch != 'q') {
        int obstacle_L, obstacle_R, obstacle; // FSM-OA
        const char blobColor[3] = { 255, 0, 0 };  // TODO: maybe adjust depending on room's light condition
        TBlobSearch blob;
        unsigned int blob_count = 0;  // simple counter of performed blob searches
        int time, time_blob;
        int blobSufficient; // FSM-SB
        int carBlobAligned; // FSM-AB
        int distance;
        enum { distok, tooclose, toofar} distanceState; // FSM-MB

        mvprintw(1, 1,"%s: Press 'q' to end program", argv[0]);
        blob.size = 0;

        obstacle_L = ( initio_IrLeft() !=0 );
        obstacle_R = ( initio_IrRight()!=0 );
        obstacle = obstacle_L || obstacle_R;

        // FSM-OA (Obstacle Avoidance)
        if (obstacle) {
            mvprintw(3, 1,"State OA (stop to avoid obstacle), o-left=%d, o-right=%d", obstacle_L, obstacle_R);
            clrtoeol(); // curses library
            initio_DriveForward (0); // Stop
        }
        else {
            // blob search is time consuming, keep some intervals between updates
            time = millis();
            if (blob_count==0 || time_blob < (time - 1000)) {
                refresh(); // curses lib: update display
                blob = cameraSearchBlob( blobColor ); // search for sign with RED colored blob
                // writeImageWithBlobAsJPEG() seems to have a bug, do not use right now:
                //writeImageWithBlobAsJPEG(blob, "test_blob.jpg", 70);  // TODO: this function is for testing (deactivate if not needed)
                time_blob = time;
                blob_count++;
            }
            blobSufficient = (blob.size > 20);  // TODO: experiment with that threshold value

            // FSM-SB (Search Blob)
            if ( ! blobSufficient ) {
                mvprintw(3, 1,"State SB (search blob), blob.size=%d (count: %u)", blob.size, blob_count);
                clrtoeol(); // curses library
                // TODO: potential actions: turn car or camera platform a few steps around and see if a blob is to be found
            }
            else {
                carBlobAligned = (blob.halign >= -0.15 && blob.halign <= 0.15);  // TODO: adjust values to useful ones

                // FSM-AB (Align to Blob)
                if ( ! carBlobAligned) {
                    mvprintw(3, 1,"State AB (align towards blob), blob.size=%d, halign=%f", blob.size, blob.halign);
                    clrtoeol(); // curses library
                    // TODO: slightly turn car to align with blob (find a useful turn duration)
                    //       if blob.halign is negative, then turn right, otherwise left
                }
                else {
                    distance = initio_UsGetDistance ();
                    if (distance < 40)
                    { distanceState = tooclose; }
                    else if (distance > 120)
                    { distanceState = toofar; }
                    else 
                    { distanceState = distok; }
 
                    // FSM-MB (cat at middle of blob, keep distance)
                    switch (distanceState) {
                    case toofar:
                        mvprintw(3, 1,"State FB (drive forward), dist=%d", distance);
                        clrtoeol(); // curses library
                        // TODO: move car forward to come closer
                        break;
                    case tooclose:
                        mvprintw(3, 1,"State RB (drive backwards), dist=%d", distance);
                        clrtoeol(); // curses library
                        // TODO: move car backwards to get more distance
                        break;
                    case distok:
                        mvprintw(3, 1,"State KD (keep distance), dist=%d", distance);
                        clrtoeol(); // curses library
                        initio_DriveForward (0); // Stop
                    } // switch (FSM-MB)
                } // if (FSM-AB)
            } // if (FSM-SB)
        } // if (FSM-OA)

        ch = getch();
        if (ch != ERR) mvprintw(2, 1,"Key code: '%c' (%d)  ", ch, ch);
        refresh(); // curses lib: update display
        //delay (100); // pause 100ms
  } // while

  return;
}


//======================================================================
// main(): initialisation of libraries, etc
//======================================================================
int main (int argc, char *argv[])
{
  WINDOW *mainwin = initscr();  // curses: init screen
  noecho ();                    // curses: prevent the key being echoed
  cbreak();                     // curses: disable line buffering 
  nodelay(mainwin, TRUE);       // curses: set getch() as non-blocking 
  keypad (mainwin, TRUE);       // curses: enable detection of cursor and other keys

  initio_Init (); // initio: init the library

  camcar(argc, argv);

  initio_Cleanup ();  // initio: cleanup the library (reset robot car)
  endwin();           // curses: cleanup the library
  return EXIT_SUCCESS;
}

