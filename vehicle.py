import time
import numpy as np
import logging
from threading import Thread
from .memory import Memory
import traceback

logger = logging.getLogger(__name__)


class Vehicle:
    def __init__(self, mem=None):

        if not mem:
            mem = Memory()

        self.mem = mem
        self.parts = []
        self.on = True
        self.threads = []

    def add(self,
            part,
            inputs=[],
            outputs=[],
            threaded=False,
            run_condition=None):
        """
        Method to add a part to the vehicle drive loop.

        Parameters
        ----------
            part: class
                donkey vehicle part has run() attribute
            inputs : list
                Channel names to get from memory.
            outputs : list
                Channel names to save to memory.
            threaded : boolean
                If a part should be run in a separate thread.
            run_condition : str
                If a part should be run or not
        """
        assert type(inputs) is list, "inputs is not a list: %r" % inputs
        assert type(outputs) is list, "outputs is not a list: %r" % outputs
        assert type(threaded) is bool, "threaded is not a boolean: %r" % threaded

        p = part
        logger.info('Adding part {}.'.format(p.__class__.__name__))
        entry = {'part': p, 'inputs': inputs, 'outputs': outputs, 'run_condition': run_condition}

        if threaded:
            t = Thread(target=part.update, args=())
            t.daemon = True
            entry['thread'] = t

        self.parts.append(entry)

    def remove(self, part):
        """
        remove part form list
        """
        self.parts.remove(part)

    def start(self, rate_hz=10, max_loop_count=None, verbose=False):
        """
        Start vehicle's main drive loop.

        This is the main thread of the vehicle. It starts all the new
        threads for the threaded parts then starts an infinite loop
        that runs each part and updates the memory.

        Parameters
        ----------

        rate_hz : int
            The max frequency that the drive loop should run. The actual
            frequency may be less than this if there are many blocking parts.
        max_loop_count : int
            Maximum number of loops the drive loop should execute. This is
            used for testing that all the parts of the vehicle work.
        verbose: bool
            If debug output should be printed into shell
        """

        try:

            self.on = True

            for entry in self.parts:
                if entry.get('thread'):
                    # start the update thread
                    entry.get('thread').start()

            # wait until the parts warm up.
            logger.info('Starting vehicle at {} Hz'.format(rate_hz))

            loop_count = 0
            while self.on:
                start_time = time.time()
                loop_count += 1

                self.update_parts()

                # stop drive loop if loop_count exceeds max_loopcount
                if max_loop_count and loop_count > max_loop_count:
                    self.on = False

                sleep_time = 1.0 / rate_hz - (time.time() - start_time)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    # print a message when could not maintain loop rate.
                    if verbose:
                        logger.info('WARN::Vehicle: jitter violation in vehicle loop '
                                    'with {0:4.0f}ms'.format(abs(1000 * sleep_time)))


        except KeyboardInterrupt:
            pass
        except Exception as e:
            traceback.print_exc()
        finally:
            self.stop()

    def update_parts(self):
        '''
        loop over all parts
        '''
        for entry in self.parts:

            run = True
            # check run condition, if it exists
            if entry.get('run_condition'):
                run_condition = entry.get('run_condition')
                run = self.mem.get([run_condition])[0]

            if run:
                # get part
                p = entry['part']
                # get inputs from memory
                inputs = self.mem.get(entry['inputs'])
                # run the part
                if entry.get('thread'):
                    outputs = p.run_threaded(*inputs)
                else:
                    outputs = p.run(*inputs)

                # save the output to memory
                if outputs is not None:
                    self.mem.put(entry['outputs'], outputs)

    def stop(self):
        logger.info('Shutting down vehicle and its parts...')
        for entry in self.parts:
            try:
                entry['part'].shutdown()
            except AttributeError:
                # usually from missing shutdown method, which should be optional
                pass
            except Exception as e:
                logger.error(e)
