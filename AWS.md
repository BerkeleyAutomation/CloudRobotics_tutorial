Before starting, you need to have an AWS account.  If you do not have one already, you can [create an AWS account](https://docs.aws.amazon.com/accounts/latest/reference/manage-acct-creating.html).  Note: this will require that you provide a credit card.  However, we will keep within the free-tier usage where possible.  If part of a formal session, you may be provided AWS credits as part of the session. 

## Create API credentials

AWS provides [instructions on how to create an API key](https://docs.aws.amazon.com/IAM/latest/UserGuide/id_credentials_access-keys.html#Using_CreateAccessKey).  Their best practices recommend using an [IAM account](https://aws.amazon.com/premiumsupport/knowledge-center/create-access-key/) first.  Feel free to follow the best practices.  However, for this bootcamp, if you create the API key for the session, then delete them at the end of the session, you should be okay.

From this process, you will need to obtain (1) an *Access key ID* (e.g., `AKIAIOSFODNN7EXAMPLE`), and (2) a *Secret Access Key* (e.g., `wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY`).  The two parts are similar to a username and password used for accessing a website, but in this case, are for accessing the interface that FogROS2 will use to talk to AWS.

(Note: API = Application Program Interface)

## Configure your AWS credentials

For this part, there are two options.

We highly recommend installing the AWS command line interface (CLI), as it can help facilitate interactions with AWS such as testing credentials.  Thus we recommend [installing the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html), and using Option 2 below.

### Option 1: Manually create a credential file

We need to create a file `~/.aws/credentials` that looks like this (but with your access key id and secret access key from the previous step)
```
[default]
aws_access_key_id=AKIAIOSFODNN7EXAMPLE
aws_secret_access_key=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
```

A quick way to do this is to open a terminal and run the following commands (but with your access key id and secret access key from the previous step):
```
mkdir -p ~/.aws
echo "[default]" > ~/.aws/credentials
echo "aws_access_key_id=AKIAIOSFODNN7EXAMPLE" >> ~/.aws/credentials
echo "aws_secret_access_key=wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY" >> ~/.aws/credentials
```
(Quick note: the `>` means create and write to the file, and `>>` means append to the file.  It is important to use `>` for the first command and `>>` for the next two.)

At this point, double check that the file looks correct, and has your access key id and secret access key from the previous step.

You can also create a simple configuration that will provide default region for AWS command.  This part is not necessary for FogROS, but can be useful for other interactions with AWS.
```
echo "[default]" > ~/.aws/config
echo "region=us-west-1" >> ~/.aws/config
```

### Option 2: Use the AWS CLI

If you have the AWS CLI (command line interface) installed, you can use it to set up the credentials that FogROS2 will use.  Having the CLI install is not required for FogROS2, however, if interested, here are the [instructions for installing the AWS CLI](https://docs.aws.amazon.com/cli/latest/userguide/getting-started-install.html).

Run `aws configure`.  You will then be prompted for your keys.  This command does exactly what the prior instructions did--creates ~/.aws/credentials and ~/.aws/config

```
% aws configure
AWS Access Key ID [None]: AKIAIOSFODNN7EXAMPLE
AWS Secret Access Key [None]: wJalrXUtnFEMI/K7MDENG/bPxRfiCYEXAMPLEKEY
Default region name [None]: us-west-1
Default output format [None]:
```
(leave output format blank)

If all went well, you should then be able to test your credentials by running the following command [1](https://stackoverflow.com/a/42241040):

```
aws sts get-caller-identity
```

You should get output similar to the following:
```
{
    "Account": "123456789012", 
    "UserId": "AR#####:#####", 
    "Arn": "arn:aws:sts::123456789012:assumed-role/role-name/role-session-name"
}
```
