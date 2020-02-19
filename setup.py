import setuptools

with open("README.md", "r") as fh:
    long_description = fh.read()

setuptools.setup(
    name='frvcpy',
    version='0.1.0rc4',
    author='Nicholas Kullman',
    author_email='nicholas.kullman@etu.univ-tours.fr',
    description='A solver for fixed route vehicle charging problems',
    long_description=long_description,
    long_description_content_type='text/markdown',
    url='https://github.com/e-VRO/frvcpy',
    packages=setuptools.find_packages(),
    install_requires=['xmltodict'],
    package_data={
        "frvcpy.test": ["data/*"],
    },
    license='Apache',
    classifiers=[
        "Programming Language :: Python :: 3",
            "License :: OSI Approved :: Apache Software License",
            "Operating System :: OS Independent",
    ],
    python_requires='>=3.6',
    entry_points={
        'console_scripts': [
            'frvcpy=frvcpy.solver:main',
            'frvcpy-translate=frvcpy.translator:main',
            'frvcpy-test=frvcpy.test.test:runAll'
        ],
    }
)
