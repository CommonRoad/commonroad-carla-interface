import os

import imageio


class Gif_Creator():
    """
    Handles the GIF creation
    """
    def __init__(self, path, gif_name):
        """

        :param path: path of the root folder for the GIF, within this directory is a folder /img containing all images saved by the ego-vehicle camera
        :param gif_name: filename of the GIF to be created
        """
        self.path = path
        self.gif_name = gif_name


    def make_gif(self):
        """
        Creates a GIF of the images provided in "path"/img - Based on https://stackoverflow.com/a/35943809 & https://pythonguides.com/python-get-all-files-in-directory/
        """
        filenames = []
        path = self.path + "/img"
        
        # Get filenames
        for root, dirs, files in os.walk(path):
            for file in files:
                if(file.endswith(".jpg")):
                    filenames.append(file)
        
        filenames.sort()
        
        # Make GIF
        with imageio.get_writer(f"{self.path}/{self.gif_name}.gif", mode='I') as writer:
            for filename in filenames:
                image = imageio.imread(os.path.join(path, filename))
                writer.append_data(image)
        
        print("GIF created!")

