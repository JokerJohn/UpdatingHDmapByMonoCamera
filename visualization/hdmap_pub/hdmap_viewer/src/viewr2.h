//
// Created by lfg on 19-8-1.
//

#ifndef HDMAP_VIEWER_VIEWR2_H
#define HDMAP_VIEWER_VIEWR2_H


class cloudHandler {

    void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                                void* viewer_void)
    {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
        if (event.keyDown())
        {
            switch (event.getKeyCode())
            {
                case ' ':    // space: grab a single frame
                    record_single = true;
                    break;

                case 'p':   // paused
                    paused = !paused;
                    break;

                case 'K':
                    record_fixed_number = false;
                    record_continuously = !record_continuously;
                    if (record_continuously)
                        std::cerr << "STARTED recording." << std::endl;
                    else
                        std::cerr << "STOPPED recording." << std::endl;
                    break;

                case 'L':
                    record_fixed_number = true;
                    record_continuously = !record_continuously;
                    if (record_continuously)
                    {
                        std::cerr << "STARTED recording." << std::endl;
                        rec_nr_frames = 0;
                    }
                    else
                        std::cerr << "STOPPED recording." << std::endl;
                    break;
            }
        }
    }

};


#endif //HDMAP_VIEWER_VIEWR2_H
